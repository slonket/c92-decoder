/// CONFIGURATION FOR MARKLIN 5-POLE MOTOR
/// 
/// NOTES:
/// - PI gains are dependent on the BEMF values and motor frequency

// GENERAL MOTOR PARAMETERS
pub const F_PWM: u32 = 25_000;  // PWM frequency (Hz)
pub const F_PID: u32 = 100;     // sampling + PI controller rate (Hz)
pub const T_BEMF: u32 = 500;    // BEMF cutout duration (us)
pub const T_ADC: u32 = 300;     // ADC conversion start time (us)
pub const N_SAMPLE: usize = 18; // number of BEMF samples (~10us each)
pub const N_REJECT: usize = 2;  // number of peak BEMF samples to reject (commutator noise suppression)

// BEMF THRESHOLDS
pub const BEMF_OFF: u16 = 15;   // BEMF value to consider the loco "stopped" for state-transition
pub const BEMF_MIN: u16 = 30;   // minimum speed value
pub const BEMF_MAX: u16 = 3760; // maximum speed value (12V BEMF from divider = 3.03V)

// GAIN SCHEDULE
pub const GAIN_BLEND_LOW: i32 = 20;   // below this = full crawl gains
pub const GAIN_BLEND_HIGH: i32 = 80; // above this = full normal gains

// PI CONTROLLER GAINS
pub const KP_NORM: i32 = 150;  // normal proportion co-efficient
pub const KI_NORM: i32 = 15;   // normal integrator co-efficient
pub const KP_CRAWL: i32 = 300; // crawl proportion co-efficient
pub const KI_CRAWL: i32 = 30;  // crawl integrator co-efficient