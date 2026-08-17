#![no_std]
#![no_main]

// CORE EMBEDDED INCLUDES
use cortex_m as _;
use cortex_m_rt as rt;
use defmt_rtt as _;
use panic_probe as _;
use stm32g0xx_hal::{self as hal, pac};

// EMBEDDED INCLUDES
use cortex_m::{
    peripheral::NVIC
};
use hal::{
    prelude::*,
    rcc::{Config, Prescaler, PllConfig},
    timer::{pins::TimerPin},
};
use pac::{interrupt, Interrupt};
use rt::{entry};

// OTHER INCLUDES
use core::{
    cell::UnsafeCell,
    mem::MaybeUninit,
};
use defmt::{info, warn};
use mm_decoder::*;
use lenz_decoder::*;

// PROJECT MODULES
mod motor_control;
mod ring_buffer;
mod decoder_state;

use decoder_state::DecoderState;
use ring_buffer::{RingBuffer, RingProducer};
use motor_control::MotorControl;

// MOTOR CONSTANTS
const F_CLK: u32 = 64_000_000;
const F_PWM: u32 = 25_000;
const F_PID: u32 = 100;

const N_DEADTIME: usize = 24;
const N_CYCLES: usize = ((F_PWM/F_PID) as usize) - N_DEADTIME;

const T_PWM: u32 = (F_CLK/F_PWM) - 1;
const T_PWM_LONG: u32 = (F_CLK/F_PWM) * ((N_DEADTIME + 1) as u32) - 1;
const T_ADC: u32 = T_PWM_LONG - ((F_CLK/1_000_000)*10); //10us before end of deadtime

// OTHER CONSTANTS
const N_PULSE_BUF: usize = 64;
const I_TRIP: u32 = 1240; // approx 1V = 1A

const ADDRESS: u8 = 10;

// TYPES
// ADC BUFFER (named u16 array)
#[repr(C)] // this ensures entries are aligned as in C - not rearranged or padded.
struct AdcBuf {
    bemf: u16,
    v_acc: u16,
    v_max: u16,
    i_motor: u16,
}

impl AdcBuf {
    const LEN: usize = 4;
}

// READ-ONLY STATICS - these will live in .rodata (flash only)
static TIM2_ARR_BUF: [u32; N_CYCLES] = {
      let mut buf: [u32; N_CYCLES] = [T_PWM; N_CYCLES];
      buf[N_CYCLES - 1] = T_PWM_LONG;
      buf
};

// READ-WRITE STATICS
#[repr(transparent)]
struct SyncCell<T>(UnsafeCell<T>);
unsafe impl<T> Sync for SyncCell<T> {}

#[unsafe(link_section = ".uninit")] // unloaded RAM section
static ADC_BUF: SyncCell<MaybeUninit<AdcBuf>> = SyncCell(UnsafeCell::new(MaybeUninit::uninit()));
#[unsafe(link_section = ".uninit")] // unloaded RAM section
static PULSE_BUF: SyncCell<MaybeUninit<RingBuffer<u16, N_PULSE_BUF>>> = SyncCell(UnsafeCell::new(MaybeUninit::uninit()));

static PULSE_PROD: SyncCell<Option<RingProducer<'static, u16, N_PULSE_BUF>>> = SyncCell(UnsafeCell::new(None));
static MOTOR_CONTROL: SyncCell<MotorControl> = SyncCell(UnsafeCell::new(MotorControl::new()));



#[entry]
fn main() -> ! {

    // track protocol decoding state machines
    static mut LENZ_MACHINE: LenzMachine = LenzMachine::new();
    static mut MM_LOCO_MACHINE: MmLocoMachine = MmLocoMachine::new();
    static mut MM_ACC_MACHINE: MmAccMachine = MmAccMachine::new();

    // decoder state machine and edge detector
    static mut DECODER_STATE: DecoderState = DecoderState::new();

    // print version at startup
    let version = env!("CARGO_PKG_VERSION");
    info!("C92 FIRMWARE VER-{}", version);

    // peripheral handles
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // clock configuration
    let mut rcc = dp.RCC.freeze(Config::pll()
        .pll_cfg(PllConfig::with_hsi(1, 8, 2)) // core clock 64MHz (16MHz HSI x8/2)
        .ahb_psc(Prescaler::NotDivided) // AHB = 64MHz
        .apb_psc(Prescaler::NotDivided) // APB = 64MHz
    );

    // gpio configuration
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // motor outputs
    let mut motor_fw = gpioa.pa2.into_push_pull_output();
    let mut motor_rv = gpioa.pa3.into_push_pull_output();
    motor_fw.set_low().ok(); // set both low to 100% prevent shoot-through
    motor_rv.set_low().ok();

    // function outputs
    let mut f0_fw = gpiob.pb6.into_open_drain_output();
    let mut f0_rv = gpiob.pb7.into_open_drain_output();
    f0_fw.set_high().ok(); // TEMP only while using open-drain outputs
    f0_rv.set_high().ok();

    // peripheral clock enable
    unsafe {
        let rcc_raw = &*pac::RCC::ptr();
        rcc_raw.ahbenr.modify(|_, w|
            w.dmaen().set_bit() // DMA
        );
        rcc_raw.apbenr1.modify(|_, w|
            w.tim2en().set_bit() // TIM2
        );
        rcc_raw.apbenr2.modify(|_, w|
            w.tim14en().set_bit() // TIM14
            .adcen().set_bit() // ADC
        );
    }

    // DMA setup
    unsafe {
        let dma = &*pac::DMA::ptr();
        let dmamux = &*pac::DMAMUX::ptr();
        let tim2 = &*pac::TIM2::ptr();
        let adc = &*pac::ADC::ptr();

        // DMA channel 1 for TIM2 ARR buffer
        dmamux.c0cr.write(|w| w.dmareq_id().bits(31)); // see Page 298 - Table 55 (31 = TIM2_UP)
        dma.ch1.par.write(|w| w.bits(&tim2.arr as *const _ as u32));
        dma.ch1.mar.write(|w| w.bits(TIM2_ARR_BUF.as_ptr() as u32));
        dma.ch1.ndtr.write(|w| w.bits(TIM2_ARR_BUF.len() as u32));
        dma.ch1.cr.write(|w|
            w.dir().set_bit() // memory to peripheral
            .minc().set_bit() // memory increment
            .pinc().clear_bit() // peripheral address fixed
            .psize().bits(0b10) // 32-bit peripheral
            .msize().bits(0b10) // 32-bit memory
            .circ().set_bit() // circular mode
            .en().set_bit() // enable channel
        );

        // DMA channel 2 for ADC result buffer
        dmamux.c1cr.write(|w| w.dmareq_id().bits(5)); // see Page 298 - Table 55 (5 = ADC)
        dma.ch2.par.write(|w| w.bits(&adc.dr as *const _ as u32));
        dma.ch2.mar.write(|w| w.bits(ADC_BUF.0.get() as u32));
        dma.ch2.ndtr.write(|w| w.bits(AdcBuf::LEN as u32));
        dma.ch2.cr.write(|w|
            w.dir().clear_bit() // peripheral to memory
            .minc().set_bit() // memory increment
            .pinc().clear_bit() // peripheral address fixed
            .psize().bits(0b01) // 16-bit peripheral
            .msize().bits(0b01) // 16-bit memory
            .circ().set_bit() // circular mode
            .tcie().set_bit() // transfer complete interrupt enable
            .en().set_bit() // enable channel
        );
    }

    // ADC setup
    unsafe {
        let adc = &*pac::ADC::ptr();

        // ADC startup procedure
        adc.cfgr2.write(|w| w.ckmode().bits(0b01)); // clock PCLK/2 = 32MHz (must be set before ADC enable)
        adc.cr.write(|w| w.advregen().set_bit()); // enable voltage regulator and wait for stabilisation (20us)
        cortex_m::asm::delay(64 * 20); // 20us delay at 64MHz - see t_ADCVREG_STUP in datasheet
        adc.cr.modify(|_, w| w.adcal().set_bit()); // start calibration (ADC must be disabled)
        while adc.cr.read().adcal().bit_is_set() {} // wait for cal complete
        adc.isr.write(|w| w.adrdy().set_bit()); // clear ADC ready
        adc.cr.modify(|_, w| w.aden().set_bit()); // enable ADC
        while adc.isr.read().adrdy().bit_is_clear() {} // wait for ADC ready
    
        // configure for trigger on TIM1_TRGO2, rising edge, DMA circular, 12-bit
        adc.cfgr1.write(|w|
            w.extsel().bits(0b010) // TRG0 = TIM2_TRGO
            .exten().bits(0b01) // rising edge trigger
            .res().bits(0b00) // 12-bit resolution
            .dmacfg().set_bit() // DMA circular mode
            .dmaen().set_bit() // DMA enable
            .chselrmod().set_bit() // sequenced channel selection (SQ1..SQ8)
        );
        while adc.isr.read().ccrdy().bit_is_clear() {} // wait for channel config ready
        adc.isr.write(|w| w.ccrdy().set_bit()); // clear ready flag
    
        // configure conversion sequence
        adc.chselr_1().write(|w|
            w.sq1().bits(5)     // motor BEMF (PA5)
            .sq2().bits(8)      // acceleration (PA8)
            .sq3().bits(1)      // maximum speed (PA1)
            .sq4().bits(7)      // motor current (PA7)
            .sq5().bits(0b1111) // 1111 = no channel and EOS
        );
        while adc.isr.read().ccrdy().bit_is_clear() {} // wait for channel config ready
        adc.isr.write(|w| w.ccrdy().set_bit()); // clear ready flag
    
        // set sampling time to 79.5 ADC clock cycles => ~2.5uS
        // TODO: Check the sampling time vs. input impedance in datasheet
        adc.smpr.write(|w| w.smp1().bits(0b101));
    
        // start (wait for TRGO2 trigger)
        adc.cr.modify(|_, w| w.adstart().set_bit());
    }

    // TIM2 setup (motor control PA0)
    unsafe {
        let gpioa = &*pac::GPIOA::ptr();
        let tim2 = &*pac::TIM2::ptr();

        // gpio setup
        gpioa.moder.modify(|_, w| w.moder0().bits(0b10)); // alternate mode for TIM2_CH1
        gpioa.afrl.modify(|_, w| w.afsel0().bits(0b0010)); // AF2 = TIM2_CH1

        // general timer config
        tim2.arr.write(|w| w.bits(T_PWM)); // set PWM frequency (25kHz)
        tim2.cr1.write(|w|
            w.arpe().set_bit() // preload ARR (prevent glitches with DMA reload)
            .urs().set_bit() // update event only at overflow
        );
        tim2.cr2.write(|w| w.mms().bits(0b101)); // OC2REF (pulse) on TRGO for ADC. See MMS in TIM2_CR2.
        tim2.dier.write(|w| w.ude().set_bit()); // update DMA request enable

        // channel configuration (CH1/CH2)
        tim2.ccmr1_output().write(|w|
            w.oc1m().bits(0b0110) // CH1 PWM mode 1 (low on match)
            .oc1pe().set_bit() // CCR1 preload enable
            .oc2m().bits(0b0111) // CH5 PWM mode 2 (high on match)
            .oc2pe().set_bit() // not strictly necessary as this is loaded once only
        );
        tim2.ccr1.write(|w| w.bits(0)); // CH1 PWM duty cycle is 0 from start - modified by motor control machine
        tim2.ccr2.write(|w| w.bits(T_ADC)); // CH2 ADC read trigger
        tim2.ccer.write(|w|
            w.cc1e().set_bit() // output CH1 (PA0)
        );

        // final counter enable - this enables the whole motor drive sequence with BEMF
        tim2.cr1.modify(|_, w| w.cen().set_bit()); // enable the counter
    }

    // TIM14 setup (track data capture PA4)
    unsafe {
        let gpioa = &*pac::GPIOA::ptr();
        let tim14 = &*pac::TIM14::ptr();

        // gpio setup (alternate mode for TIM14_CH1)
        gpioa.moder.modify(|_, w| w.moder4().bits(0b10));
        gpioa.afrl.modify(|_, w| w.afsel4().bits(0b0100)); // AF4 = TIM14_CH1

        tim14.dier.write(|w| w.cc1ie().set_bit()); // enable CC1 interrupt
        tim14.ccmr1_input().write(|w|
            w.ic1f().bits(0b0010) // 0011 N=8, 0010 N=4
            .cc1s().bits(0b01) // CC1 input capture
        );
        tim14.ccer.write(|w|
            w.cc1np().set_bit() // CC1NP=1 + CC1P=1 = both edges
            .cc1p().set_bit()
            .cc1e().set_bit() // input capture enabled
        );
        tim14.psc.write(|w| w.bits(63u32)); // 1MHz counter frequency (1us resolution)
        tim14.cr1.write(|w| w.cen().set_bit()); // enable the counter
    }

    // pulse buffer setup
    let pulse_buf = unsafe { &mut *PULSE_BUF.0.get() };
    pulse_buf.write(RingBuffer::new());
    let (pulse_prod, mut pulse_cons) = unsafe { pulse_buf.assume_init_mut() }.split();
    unsafe { *PULSE_PROD.0.get() = Some(pulse_prod); }

    // motor control setup
    let motor_control = unsafe {
        let mc = &mut *MOTOR_CONTROL.0.get();
        let tim2 = &*pac::TIM2::ptr();
        mc.init(motor_fw, motor_rv, tim2);
        mc
    };

    // decoder state setup
    DECODER_STATE.init(f0_fw, f0_rv);

    // set interrupt priorities and enable
    let mut nvic = cp.NVIC;
    unsafe {
        nvic.set_priority(Interrupt::DMA_CHANNEL2_3, 0b11000000); // lowest priority (3)
        nvic.set_priority(Interrupt::TIM14, 0b00000000); // highest priority (0)
        NVIC::unmask(Interrupt::DMA_CHANNEL2_3);
        NVIC::unmask(Interrupt::TIM14);
    }

    loop {

        // check and process pulses - this will skip other loop items beyond the state machines as well
        if let Ok(pulse) = pulse_cons.get() {

            // processing Lenz protocol
            if let Some(packet) = LENZ_MACHINE.advance(pulse) {
                match packet.get_type() {
                    Some(LenzCommand::Speed(s)) if s.address() == ADDRESS => {
                        // update f0
                        DECODER_STATE.update_f0(s.f0());

                        // update direction
                        if DECODER_STATE.update_direction(s.direction()) {
                            motor_control.new_direction(s.direction());
                        }

                        // update speed
                        if let LenzSpeed::Speed(speed) = s.speed() {
                            if DECODER_STATE.update_speed(speed) {
                                motor_control.new_speed(speed);
                            }
                        }
                    }
                    Some(LenzCommand::Function(f)) if f.address() == ADDRESS => {
                        let states = f.states();
                        // f1.set_state(states[0].into()).unwrap();
                        // f2.set_state(states[1].into()).unwrap();
                        // f3.set_state(states[2].into()).unwrap();
                        // f4.set_state(states[3].into()).unwrap();
                        motor_control.ramp_bypass(states[3]);
                    }
                    _ => {}
                }
            }

            // processing MM loco protocol
            if let Some(packet) = MM_LOCO_MACHINE.advance(pulse) {

                // ignore packets for foreign addresses
                if packet.ext_address() == ADDRESS {

                    // update f0 - present in every packet
                    DECODER_STATE.update_f0(packet.f0());

                    // update command
                    match packet.command() {
                        MmLocoCommand::OldSpeed(MmSpeed::Speed(speed)) => {
                            if DECODER_STATE.update_speed(speed) {
                                motor_control.new_speed(speed);
                            }
                        }
                        MmLocoCommand::OldSpeed(MmSpeed::Reverse) => {
                            if let Some(direction) = DECODER_STATE.update_reverse() {
                                motor_control.new_direction(direction);
                                // changing direction is independent of speed at the packet level, thus
                                // the speed needs to also be set to 0 to prevent "restarting" in the
                                // new direction.
                                motor_control.new_speed(0);
                            }
                        }
                        MmLocoCommand::NewSpeed { speed: MmSpeed::Speed(speed), direction } => {
                            if DECODER_STATE.update_direction(direction) {
                                motor_control.new_direction(direction);
                            }
                            if DECODER_STATE.update_speed(speed) {
                                motor_control.new_speed(speed);
                            }
                        }
                        MmLocoCommand::Function { speed: MmSpeed::Speed(speed), function, state } => {
                            if DECODER_STATE.update_speed(speed) {
                                motor_control.new_speed(speed);
                            }

                            // set the corresponding function - only one per function packet
                            match function {
                                1 => {} // f1.set_state(state.into()).unwrap(),
                                2 => {} // f2.set_state(state.into()).unwrap(),
                                3 => {} // f3.set_state(state.into()).unwrap(),
                                4 => {
                                    // f4.set_state(state.into()).unwrap();
                                    motor_control.ramp_bypass(state);
                                }
                                _ => {}
                            }
                        }
                        _ => {} // MM2 reverse commands are ignored - TODO maybe not?
                    }
                }
            }

            // processing MM accessory (old function) protocol
            if let Some(packet) = MM_ACC_MACHINE.advance(pulse) {
                if let MmAccCommand::Func(f) = packet.get_type() {

                    // ignore packets for foreign addresses
                    if f.ext_address() == ADDRESS {

                        // update all functions
                        let states = f.states();
                        // f1.set_state(states[0].into()).unwrap();
                        // f2.set_state(states[1].into()).unwrap();
                        // f3.set_state(states[2].into()).unwrap();
                        // f4.set_state(states[3].into()).unwrap();
                        motor_control.ramp_bypass(states[3]);
                    }
                }
            }
        }

        // anything else to do in main loop? put it here :)
    }
}

#[interrupt]
fn DMA_CHANNEL2_3() {

    // local statics
    static mut DECIMATOR: u8 = 0;
    static mut ACC_CUM: u16 = 0;
    static mut VMAX_CUM: u16 = 0;

    static mut BEMF_IIR: i32 = 0;

    // global static handles
    let dma = unsafe { &*pac::DMA::ptr() };
    let dma_buf = unsafe { (&*ADC_BUF.0.get()).assume_init_ref() };
    let motor_control = unsafe { &mut *MOTOR_CONTROL.0.get() };

    // clear interrupt flag
    dma.ifcr.write(|w| w.ctcif2().set_bit());

    // BEMF IIR filter - set to 1/2 (shift 1)
    // this is necessary with the marklin 5-pole DCM motor. Compared to finer DC motors, the mechanical gear drive
    // of models with this motor causes a lot of noise on the BEMF signal. What is being read is the true BEMF value
    // at any given instant, but it causes a lot of instability in the PI controller. Thus, it's software filtered. Note
    // the BEMF dividers also have a hardware RC filter for commutator/general noise.
    *BEMF_IIR += ((dma_buf.bemf as i32) - *BEMF_IIR) >> 1;

    // update the motor machine
    motor_control.tick(*BEMF_IIR as u16);
    
    // updating the config - 8 sample average
    if *DECIMATOR >= 7 {
        // divide both by 8
        let acc = *ACC_CUM >> 3;
        let vmax = *VMAX_CUM >> 3;

        // update machine config variables
        motor_control.new_config(acc, vmax);

        // reset accumulators and decimator - store current values for next 8
        *ACC_CUM = dma_buf.v_acc;
        *VMAX_CUM = dma_buf.v_max;
        *DECIMATOR = 0;
    } else {
        // add values this cycle and store
        *ACC_CUM += dma_buf.v_acc;
        *VMAX_CUM += dma_buf.v_max;
        *DECIMATOR += 1;
    }
}

#[interrupt]
fn TIM14() {

    static mut LAST_CCR1: u16 = 0;

    let tim14 = unsafe { &*pac::TIM14::ptr() };
    let gpioa = unsafe { &*pac::GPIOA::ptr() };
    let prod = unsafe { &mut *PULSE_PROD.0.get() }.as_mut().unwrap();

    // read the new edge value (also clears interrupt flag) and status register
    let ccr1 = tim14.ccr1.read().bits() as u16;
    let sr = tim14.sr.read();

    // check for overcapture - 0 if OVC, pulse 
    let raw_pulse = if sr.cc1of().bit_is_set() {
        tim14.sr.write(|w| unsafe { w.bits(!0) }.cc1of().clear_bit());
        warn!("Overcapture occured!");
        0
    } else {
        ccr1.wrapping_sub(*LAST_CCR1)
    };

    // applying asymmetry correction due to level shifter
    const ASYM_COMP_US: u16 = 3;
    let pulse = if raw_pulse == 0 {
        0
    } else if gpioa.idr.read().idr4().bit_is_set() {
        raw_pulse.saturating_sub(ASYM_COMP_US)
    } else {
        raw_pulse.saturating_add(ASYM_COMP_US)
    };

    // push to buffer, reset if overflow occurs
    if prod.put(pulse).is_err() {
        prod.reset();
        let _ = prod.put(0); // put sentinel "reset" for state machines
        warn!("Pulse buffer overflow!");
    }

    // store the new edge for next ISR
    *LAST_CCR1 = ccr1;
}