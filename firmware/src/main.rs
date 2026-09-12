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
};
use pac::{interrupt, Interrupt};
use rt::{entry};

// DECODER INCLUDES
#[cfg(feature = "mm")]
use mm_decoder::*;
#[cfg(feature = "lenz")]
use lenz_decoder::*;

#[cfg(not(any(feature = "mm", feature = "lenz")))]
compile_error!("At least one protocol (\"mm\" or \"lenz\") must be enabled");

// OTHER INCLUDES
use core::{
    cell::UnsafeCell,
    mem::MaybeUninit,
};
use defmt::{info, warn};

// PROJECT MODULES
mod motor_control;
mod ring_buffer;
mod decoder_state;

use decoder_state::DecoderState;
use ring_buffer::{RingBuffer, RingProducer};
use motor_control::MotorControl;

// MOTOR CONTROL

// configurable variables
const F_PWM: u32 = 25_000;
const F_PID: u32 = 100;
const T_BEMF: u32 = 1000; // BEMF cutout duration (us)
const T_ADC: u32 = 750; // ADC conversion start time (us)
const N_SAMPLE: usize = 18; // number of BEMF samples to take (~10us each)
const N_REJECT: usize = 2; // number of peak BEMF samples to reject (commutator noise suppression)

// calculated constants
const F_CLK: u32 = 64_000_000;
const TIM16_ARR: u32 = (F_CLK/F_PID)/64 - 1;
const TIM2_ARR_PWM: u32 = (F_CLK/F_PWM) - 1;
const TIM2_ARR_BEMF: u32 = (F_CLK/1_000_000) * T_BEMF - 1;
const TIM2_ADC_TRIG: u32 = TIM2_ARR_BEMF - ((F_CLK/1_000_000) * (T_BEMF - T_ADC));
const PWM_MAX: i32 = TIM2_ARR_PWM as i32;

// static value for TIM2 BEMF measurement automation
static TIM2_ARR_DMA: u32 = TIM2_ARR_BEMF;

// OTHER CONSTANTS
const N_PULSE_BUF: usize = 64;

// TYPES
// ADC BUFFER (named u16 array)
#[repr(C)] // this ensures entries are aligned as in C - not rearranged or padded.
struct BemfBuf ( [u16; N_SAMPLE] );
impl BemfBuf { const LEN: usize = N_SAMPLE; }

// READ-WRITE STATICS
#[repr(transparent)]
struct SyncCell<T>(UnsafeCell<T>);
unsafe impl<T> Sync for SyncCell<T> {}

#[unsafe(link_section = ".uninit")] // unloaded RAM section
static BEMF_BUF: SyncCell<MaybeUninit<BemfBuf>> = SyncCell(UnsafeCell::new(MaybeUninit::uninit()));
#[unsafe(link_section = ".uninit")] // unloaded RAM section
static PULSE_BUF: SyncCell<MaybeUninit<RingBuffer<u16, N_PULSE_BUF>>> = SyncCell(UnsafeCell::new(MaybeUninit::uninit()));

static PULSE_PROD: SyncCell<Option<RingProducer<'static, u16, N_PULSE_BUF>>> = SyncCell(UnsafeCell::new(None));
static MOTOR_CONTROL: SyncCell<MotorControl<PWM_MAX>> = SyncCell(UnsafeCell::new(MotorControl::new()));



#[entry]
fn main() -> ! {

    // track protocol decoding state machines
    #[cfg(feature = "lenz")]
    static mut LENZ_MACHINE: LenzMachine = LenzMachine::new();
    #[cfg(feature = "mm")]
    static mut MM_LOCO_MACHINE: MmLocoMachine = MmLocoMachine::new();
    #[cfg(feature = "mm")]
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
    let gpioc = dp.GPIOC.split(&mut rcc);

    // motor outputs
    let mut motor_fw = gpiob.pb3.into_push_pull_output();
    let mut motor_rv = gpiob.pb4.into_push_pull_output();
    motor_fw.set_low().ok(); // set both low to 100% prevent shoot-through
    motor_rv.set_low().ok();

    // function outputs
    // let f0_fw = gpioa.pa12.into_push_pull_output();
    // let f0_rv = gpiob.pb5.into_push_pull_output();
    let f0_fw = gpiob.pb5.into_push_pull_output();
    let f0_rv = gpioa.pa12.into_push_pull_output();
    let mut f1 = gpioa.pa2.into_push_pull_output();
    let mut f2 = gpioa.pa3.into_push_pull_output();
    let mut f3 = gpioa.pa0.into_push_pull_output();
    let mut f4 = gpioa.pa1.into_push_pull_output();

    // address input configuration
    let a7_pin = gpioc.pc14.into_pull_up_input();
    let a6_pin = gpiob.pb8.into_pull_up_input();
    let a5_pin = gpiob.pb6.into_pull_up_input();
    let a4_pin = gpiob.pb1.into_pull_up_input();
    let a3_pin = gpioc.pc15.into_pull_up_input();
    let a2_pin = gpioa.pa8.into_pull_up_input();
    let a1_pin = gpioc.pc6.into_pull_up_input();
    let a0_pin = gpioa.pa11.into_pull_up_input();

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
            .tim16en().set_bit() // TIM16
            .adcen().set_bit() // ADC
        );
    }

    // DMA setup
    unsafe {
        let dma = &*pac::DMA::ptr();
        let dmamux = &*pac::DMAMUX::ptr();
        let tim2 = &*pac::TIM2::ptr();
        let adc = &*pac::ADC::ptr();

        // DMA channel 1 for ADC result buffer
        dmamux.c0cr.write(|w| w.dmareq_id().bits(5)); // see Page 298 - Table 55 (5 = ADC)
        dma.ch1.par.write(|w| w.bits(adc.dr.as_ptr() as u32));
        dma.ch1.mar.write(|w| w.bits(BEMF_BUF.0.get() as u32));
        dma.ch1.ndtr.write(|w| w.bits(BemfBuf::LEN as u32));
        dma.ch1.cr.write(|w|
            w.dir().clear_bit() // peripheral to memory
            .minc().set_bit() // memory increment
            .pinc().clear_bit() // peripheral address fixed
            .psize().bits(0b01) // 16-bit peripheral
            .msize().bits(0b01) // 16-bit memory
            .circ().clear_bit() // one-shot (needs retriggering)
            .tcie().set_bit() // transfer complete interrupt enable
            .en().set_bit() // enable channel
        );

        // DMA channel 2 for TIM2 ARR one-shot
        dmamux.c1cr.write(|w| w.dmareq_id().bits(46)); // see Page 298 - Table 55 (46 = TIM16_UP)
        dma.ch2.par.write(|w| w.bits(tim2.arr.as_ptr() as u32));
        dma.ch2.mar.write(|w| w.bits(&TIM2_ARR_DMA as *const _ as u32));
        dma.ch2.ndtr.write(|w| w.bits(1));
        dma.ch2.cr.write(|w|
            w.dir().set_bit() // memory to peripheral
            .minc().clear_bit() // memory address fixed
            .pinc().clear_bit() // peripheral address fixed
            .psize().bits(0b10) // 32-bit peripheral
            .msize().bits(0b10) // 32-bit memory
            .circ().set_bit() // circular mode
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
    
        // configure for trigger on TIM2_TRGO, rising edge, DMA circular, 12-bit
        adc.cfgr1.write(|w|
            w.extsel().bits(0b010) // TRG0 = TIM2_TRGO
            .exten().bits(0b01) // rising edge trigger
            .res().bits(0b00) // 12-bit resolution
            .dmacfg().clear_bit() // one-shot DMA mode
            .cont().set_bit() // continuous ADC mode
            .dmaen().set_bit() // DMA enable
            .chselrmod().set_bit() // sequenced channel selection (SQ1..SQ8)
        );
        while adc.isr.read().ccrdy().bit_is_clear() {} // wait for channel config ready
        adc.isr.write(|w| w.ccrdy().set_bit()); // clear ready flag
    
        // configure conversion sequence
        adc.chselr_1().write(|w|
            w.sq1().bits(11)     // BEMF (PB7 = CH11)
            .sq2().bits(0b1111) // 1111 = no channel and EOS
        );
        while adc.isr.read().ccrdy().bit_is_clear() {} // wait for channel config ready
        adc.isr.write(|w| w.ccrdy().set_bit()); // clear ready flag
    
        // set sampling time to 160.5 ADC clock cycles (0b111) => ~5uS
        // 0b111 = 160.5 = ~5us -> 16 samples = 79.5us
        // 0b110 = 79.5 = ~2.5us -> 16 samples = 39.75us
        adc.smpr.write(|w| w.smp1().bits(0b111));
    
        // start (wait for TRGO2 trigger)
        adc.cr.modify(|_, w| w.adstart().set_bit());
    }

    // TIM2 setup (motor control PA0)
    unsafe {
        let gpioa = &*pac::GPIOA::ptr();
        let tim2 = &*pac::TIM2::ptr();

        // gpio setup (PA15)
        gpioa.moder.modify(|_, w| w.moder15().bits(0b10)); // alternate mode for TIM2_CH1
        gpioa.afrh.modify(|_, w| w.afsel15().bits(0b0010)); // AF2 = TIM2_CH1

        // general timer config
        tim2.arr.write(|w| w.bits(TIM2_ARR_PWM)); // set PWM frequency (25kHz)
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
            .oc2m().bits(0b0111) // CH2 PWM mode 2 (high on match)
            .oc2pe().set_bit() // not strictly necessary as this is loaded once only
        );
        tim2.ccr1.write(|w| w.bits(0)); // CH1 PWM duty cycle is 0 from start - modified by motor control machine
        tim2.ccr2.write(|w| w.bits(TIM2_ADC_TRIG)); // CH2 ADC trigger
        tim2.ccer.write(|w|
            w.cc1e().set_bit() // output CH1 (PA0)
        );

        // enable the counter
        tim2.cr1.modify(|_, w| w.cen().set_bit()); // enable the counter
    }

    // TIM16 setup (motor control measurement driver)
    unsafe {
        let tim16 = &*pac::TIM16::ptr();

        // general timer setup
        tim16.psc.write(|w| w.bits(63u32)); // 1MHz counter
        tim16.arr.write(|w| w.bits(TIM16_ARR)); // PID reload duration
        tim16.cr1.write(|w| w.urs().set_bit()); // update event only on overflow
        tim16.dier.write(|w| w.ude().set_bit()); // overflow DMA request enable
        
        // enable the counter
        tim16.cr1.modify(|_, w| w.cen().set_bit());
    }

    // TIM14 setup (track data capture PA4)
    unsafe {
        let tim14 = &*pac::TIM14::ptr();
        let gpioa = &*pac::GPIOA::ptr();

        // gpio setup (alternate mode for TIM14_CH1)
        gpioa.moder.modify(|_, w| w.moder4().bits(0b10));
        gpioa.afrl.modify(|_, w| w.afsel4().bits(0b0100)); // AF4 = TIM14_CH1

        // timer setup
        tim14.psc.write(|w| w.bits(63u32)); // 1MHz counter frequency (1us resolution)
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

        // enable the counter
        tim14.cr1.write(|w| w.cen().set_bit());
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
        nvic.set_priority(Interrupt::DMA_CHANNEL1, 0b11000000); // lowest priority (3)
        nvic.set_priority(Interrupt::TIM14, 0b00000000); // highest priority (0)
        NVIC::unmask(Interrupt::DMA_CHANNEL1);
        NVIC::unmask(Interrupt::TIM14);
    }

    // load address
    let address = {
        // this is done separately to GPIO config to allow for the inputs to stabilise
        // after the pull-ups are enabled. The ADC startup delay + other config should give a
        // sufficient delay for configuration
        let a7 = a7_pin.is_low().unwrap() as u8;
        let a6 = a6_pin.is_low().unwrap() as u8;
        let a5 = a5_pin.is_low().unwrap() as u8;
        let a4 = a4_pin.is_low().unwrap() as u8;
        let a3 = a3_pin.is_low().unwrap() as u8;
        let a2 = a2_pin.is_low().unwrap() as u8;
        let a1 = a1_pin.is_low().unwrap() as u8;
        let a0 = a0_pin.is_low().unwrap() as u8;

        a0 | a1 << 1 | a2 << 2 | a3 << 3 |
        a4 << 4 | a5 << 5 | a6 << 6 | a7 << 7
    };

    loop {

        // check and process pulses - this will skip other loop items beyond the state machines as well
        if let Ok(pulse) = pulse_cons.get() {

            // processing Lenz protocol
            #[cfg(feature = "lenz")]
            if let Some(packet) = LENZ_MACHINE.advance(pulse) {
                match packet.get_type() {
                    Some(LenzCommand::Speed(s)) if s.address() == address => {
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
                    Some(LenzCommand::Function(f)) if f.address() == address => {
                        let states = f.states();
                        f1.set_state(states[0].into()).unwrap();
                        f2.set_state(states[1].into()).unwrap();
                        f3.set_state(states[2].into()).unwrap();
                        f4.set_state(states[3].into()).unwrap();
                        motor_control.ramp_bypass(states[3]);
                    }
                    _ => {}
                }
            }

            // processing MM loco protocol
            #[cfg(feature = "mm")]
            if let Some(packet) = MM_LOCO_MACHINE.advance(pulse) {

                // ignore packets for foreign addresses
                if packet.ext_address() == address {

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
                                1 => f1.set_state(state.into()).unwrap(),
                                2 => f2.set_state(state.into()).unwrap(),
                                3 => f3.set_state(state.into()).unwrap(),
                                4 => {
                                    f4.set_state(state.into()).unwrap();
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
            #[cfg(feature = "mm")]
            if let Some(packet) = MM_ACC_MACHINE.advance(pulse) {
                if let MmAccCommand::Func(f) = packet.get_type() {

                    // ignore packets for foreign addresses
                    if f.ext_address() == address {

                        // update all functions
                        let states = f.states();
                        f1.set_state(states[0].into()).unwrap();
                        f2.set_state(states[1].into()).unwrap();
                        f3.set_state(states[2].into()).unwrap();
                        f4.set_state(states[3].into()).unwrap();
                        motor_control.ramp_bypass(states[3]);
                    }
                }
            }
        }

        // anything else to do in main loop? put it here :)
    }
}

#[interrupt]
fn DMA_CHANNEL1() {

    // global static handles
    let dma = unsafe { &*pac::DMA::ptr() };
    let adc = unsafe { &*pac::ADC::ptr() };
    let tim2 = unsafe { &*pac::TIM2::ptr() };
    let bemf_buf = unsafe { (&mut *BEMF_BUF.0.get()).assume_init_mut() };
    let motor_control = unsafe { &mut *MOTOR_CONTROL.0.get() };

    // clear interrupt flag
    dma.ifcr.write(|w| w.ctcif1().set_bit());

    // reset TIM2 to normal state
    tim2.arr.write(|w| unsafe { w.bits(TIM2_ARR_PWM) } );

    // re-arm the ADC for next cycle (stopped by DMACFG=0 one-shot completion)
    adc.cr.modify(|_, w| w.adstart().set_bit());

    // re-arm the DMA channel
    dma.ch1.cr.modify(|_, w| w.en().clear_bit() );
    while dma.ch1.cr.read().en().bit_is_set() {}
    dma.ch1.ndtr.write(|w| unsafe { w.bits(BemfBuf::LEN as u32) } );
    dma.ch1.cr.modify(|_, w| w.en().set_bit() );

    // selection sort highest N_REJECT samples
    for i in 0..N_REJECT {
        // find index of largest value
        let mut max_idx = 0;
        for j in 0..(N_SAMPLE-i) {
            if bemf_buf.0[j] > bemf_buf.0[max_idx] {
                max_idx = j;
            }
        }
        // copy current end into max sample position (discard the max)
        bemf_buf.0[max_idx] = bemf_buf.0[(N_SAMPLE - 1) - i];
    }

    // average the samples up to N_REJECT
    let mut bemf: u32 = 0;
    for i in 0..N_SAMPLE-N_REJECT {
        bemf = bemf + (bemf_buf.0[i] as u32);
    }
    bemf = bemf/((N_SAMPLE - N_REJECT) as u32);

    // update the motor machine
    motor_control.tick(bemf as u16);
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
    const ASYM_COMP_US: u16 = 5;
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