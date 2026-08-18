use stm32g0xx_hal::{
    prelude::OutputPin,
    pac::tim2,
    gpio::{PA1, PA2, Output, PushPull},
};
use core::ptr::write_volatile;

/// MOTOR CONTROLLER
/// There are many factors that can be used to tune this controller, some that are outside of its constants.
/// Here are the internal configurations related directly to the PI controller:
/// - PI_KI - Integrator coefficient.
/// - PI_KP - Proportion coefficient.
/// - I_PRELOAD - Integrator kick-start value for starting from 0. This must be adjusted after KI/KP for smooth start.
/// 
/// The BEMF value is also filtered twice - first in hardware using a divider-RC filter, and then again in software using
/// an IIR filter. Since BEMF is measured against fixed ranges in software (e.g. 3200 maximum), the maximum speed that can
/// be measured and achieved is thus set by the BEMF divider. Currently there is an approximate 1/4 divider with 100nF filter
/// capacitor.
/// 
/// The motor is very noisy at high speed - so much so that it's difficult to get stable measurements, hence the large filter capacitor.
/// Two factors can be adjusted for the BEMF hardware filtering:
/// - N_DEADTIME in main.rs - number of "dead" PWM cycles used for BEMF measurement settling.
/// - C_filter - the capacitor in the divider-RC filter.
/// 
/// A larger filter cap sets a lower cutoff frequency and rejects more noise, however this also extends BEMF settling time.
/// Thus N_DEADTIME must also be suitable. N_DEADTIME should be as short as possible so as not to affect drive strength.
/// 
/// Lastly, there is an IIR filter for BEMF samples. This should also be set to as little as possible - a larger value
/// increases phase delay, which makes the controller laggy. It can cause oscillation if too long. This can be changed
/// in the DMA_CHANNEL2_3 ISR.

// BEMF CONTROL CONSTANTS
const BEMF_OFF: u16 = 20; // BEMF value to consider the loco "stopped" for state-transition
const BEMF_MIN: u16 = 40; // minimum speed value
const BEMF_MAX: u16 = 3760; // maximum speed value (12V BEMF from divider = 3.03V)
const BEMF_LUT: [u16; 14] = { // LUT of BEMF values from speed settings

    // variables
    let base: u64 = 1150; // exponential base (x1000)

    // array for raw exponential curve
    let mut exp_raw = [1_000_000u64; 14];

    // calculate each value
    let mut i = 1;
    while i < 14 {
        // the base is multiplied by 1000 (e.g. 1.15 = 1150)
        // multiply by base then divide by 1000 -> this is an integer version of
        // directly multiplying by what would be the float base (with 3 digit precision)
        exp_raw[i] = (exp_raw[i-1] * base) / 1000;
        i += 1;
    }

    // scale each value from 0 to 65535 (u16 full range)
    let mut lut: [u16; 14] = [0; 14];
    let mut i = 0;
    while i < 14 {
        lut[i] = (((exp_raw[i] - exp_raw[0]) * 65535) / (exp_raw[13] - exp_raw[0])) as u16;
        i += 1;
    }

    lut
};

// PI CONTROLLER CONSTANTS
const PI_KP: i32 = 1024; // proportion co-efficient
const PI_KI: i32 = 64; // integrator co-efficient
const PI_SHIFT: u8 = 10; // fixed-point arithmatic scaling factor (64 fractional values)
const PI_MAX: i32 = 2559; // maximum PWM CCR1 value from PI controller
const PI_MIN: i32 = 0; // minimum PWM CCR1 value from PI controllers
const I_PRELOAD: i32 = 128_000; // kickstart value from stand-still (idle)

// MOTOR CONTROL
enum MotorState {
    Idle,
    Run,
    Brake
}

#[derive(PartialEq, Clone, Copy)]
enum Direction {
    Forward,
    Reverse
}

impl From<bool> for Direction {
    fn from(value: bool) -> Self {
        match value {
            false => Direction::Reverse,
            true => Direction::Forward,
        }
    }
}

pub struct MotorControl {

    // target BEMF calculation variables
    speed: u8,
    bemf_max: u16,
    bemf_target: u16, // target value for BEMF to reach
    bemf_setpoint: u16, // ramped value approaching bemf_target to simulate acceleration
    pot_acc: u16,

    // PI controller variables
    pi_integral: i32,

    // state machine variables
    state: MotorState,
    direction: Direction,
    pending_direction: Direction,
    ramp_bypass: bool,

    // controllable outputs
    motor_fw: Option<PA1<Output<PushPull>>>,
    motor_rv: Option<PA2<Output<PushPull>>>,
    motor_pwm: Option<*mut u32>,
}

impl MotorControl {

    /// Constructor for static initialisation
    pub const fn new() -> Self {
        Self {
            speed: 0,
            bemf_max: 0,
            bemf_target: 0,
            bemf_setpoint: 0,
            pot_acc: 0,
            pi_integral: 0,
            state: MotorState::Idle,
            direction: Direction::Forward,
            pending_direction: Direction::Forward,
            ramp_bypass: false,
            motor_fw: None,
            motor_rv: None,
            motor_pwm: None,
        }
    }

    /// Set the runtime values (pins, CCR1). MUST BE CALLED BEFORE tick()
    pub fn init(&mut self, motor_fw: PA1<Output<PushPull>>, motor_rv: PA2<Output<PushPull>>, tim2: &tim2::RegisterBlock) {
        self.motor_fw = Some(motor_fw);
        self.motor_rv = Some(motor_rv);
        self.motor_pwm = Some(&tim2.ccr1 as *const _ as *mut u32);
    }

    /// Progress the motor state machine and PI controller (if running)]. init() MUST BE CALLED FIRST. This
    /// configures the run-time pins and register which this machine controls.
    pub fn tick(&mut self, bemf: u16) {

        // handles for the motor direction GPIO
        let motor_fw = unsafe { self.motor_fw.as_mut().unwrap_unchecked() };
        let motor_rv = unsafe { self.motor_rv.as_mut().unwrap_unchecked() };
        let motor_pwm = unsafe { self.motor_pwm.unwrap_unchecked() };

        // setpoint ramping - pot_acc (0-4095) controls ramp rate per tick
        // if bypass is on, this is set to the maximum (1 step/tick);
        // TODO revise the ramp step to be non-linear
        let ramp_step = match self.ramp_bypass {
            false => {
                1 + (self.pot_acc / 64) as u16 // 1-64 BEMF units per tick
            }
            true => 64
        };

        // move setpoint towards bemf_target - this is an idiom using saturating operations
        self.bemf_setpoint = self.bemf_target.clamp(
            self.bemf_setpoint.saturating_sub(ramp_step),
            self.bemf_setpoint.saturating_add(ramp_step),
        );

        // resolve state
        match self.state {
            MotorState::Idle => {
                // check for run transition case
                if self.bemf_target > 0 {
                    match self.pending_direction {
                        Direction::Forward => {
                            //enable forward direction output
                            motor_fw.set_high().ok();
                        }
                        Direction::Reverse => {
                            //enable reverse direction output
                            motor_rv.set_high().ok();
                        }
                    }
                    // pre-load the integrator for kick-start from zero speed
                    self.pi_integral = I_PRELOAD;
                    // also preload the setpoint to be the minimum - we don't want to ramp UP from zero
                    // ramping from zero messes with the kickstart above
                    self.bemf_setpoint = BEMF_MIN;
                    // update running direction and transition to run
                    self.direction = self.pending_direction;
                    self.state = MotorState::Run;
                }
            }
            MotorState::Run => {

                // check for the idle state transition case
                if (self.bemf_setpoint == 0) && (bemf < BEMF_OFF) { // TODO this is just to test open-loop control
                //if (self.bemf_setpoint == 0) && (bemf_diff < BEMF_OFF) {
                    // turn off both direction outputs (saves checking which one to turn off)
                    motor_fw.set_low().ok();
                    motor_rv.set_low().ok();
                    // set PWM and PI integral to 0
                    unsafe { write_volatile(motor_pwm, 0) };
                    self.pi_integral = 0;
                    // transition to idle
                    self.state = MotorState::Idle;
                    return;
                }

                // check for brake transition case
                if self.pending_direction != self.direction {
                    // direction has changed while running, need to emergency brake
                    // set PWM and PI integral to 0
                    unsafe { write_volatile(motor_pwm, 0) };
                    self.pi_integral = 0;
                    // transition to brake
                    self.state = MotorState::Brake;
                    return;
                }

                // PI CONTROLLER
                let error = self.bemf_setpoint as i32 - bemf as i32;
                
                // integrate with direct clamp anti-windup
                self.pi_integral = (self.pi_integral + error * PI_KI).clamp(PI_MIN << PI_SHIFT, PI_MAX << PI_SHIFT);

                // compute final output
                let output = ((error * PI_KP + self.pi_integral) >> PI_SHIFT).clamp(PI_MIN, PI_MAX) as u32;
                
                unsafe { write_volatile(motor_pwm, output); }
            }
            MotorState::Brake => {
                // waiting for motor speed to hit "0"-ish. PWM is 0 already
                if bemf < BEMF_OFF {
                    // braking complete - turn off both direction outputs.
                    motor_fw.set_low().ok();
                    motor_rv.set_low().ok();
                    // transition to idle
                    self.state = MotorState::Idle;
                }
            }
        }
    }

    /// Update information from a new speed packet. A new speed means a new BEMF target which
    /// must be calculated using the existing vmax value.
    pub fn new_speed(&mut self, speed: u8) {
        self.speed = speed;
        self.bemf_target = Self::new_target(self.speed, self.bemf_max);
    }

    /// set a new pending direction
    pub fn new_direction(&mut self, dir: bool) {
        self.pending_direction = dir.into();
    }

    /// Update the potentiometer configuration values for acc and vmax.
    pub fn new_config(&mut self, acc: u16, vmax: u16) {
        self.pot_acc = acc;

        // vmax config
        let vmax_scaled = (vmax as u32 * 4 + 4095) / 5; // 20%-100% of pot range
        self.bemf_max = BEMF_MIN + ((vmax_scaled * (BEMF_MAX - BEMF_MIN) as u32) / 4095) as u16;
        // a new vmax config means the BEMF scale could have changed - recalculate
        self.bemf_target = Self::new_target(self.speed, self.bemf_max);
    }

    /// calculates a scaled target speed based on Vmax and LUT
    fn new_target(speed: u8, bemf_max: u16) -> u16 {
        // 0 speed must always be 0 bemf - no LUT entry needed
        if speed == 0 {
            return 0;
        }
        // index must be shifted one space and limited
        let index = (speed.min(14) - 1) as usize;
        // calculate and return scaled speed. This works because bemf_max will be within 0-4095 at most
        // due to ADC resolution. The computations fit within u32.
        let range = bemf_max.saturating_sub(BEMF_MIN) as u32;
        let new_target = BEMF_MIN + ((BEMF_LUT[index] as u32 * range) / 65535) as u16;

        new_target
    }

    /// set the ramp bypass for acceleration/braking off
    pub fn ramp_bypass(&mut self, bypass: bool) {
        self.ramp_bypass = bypass;
    }
}