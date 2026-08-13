use stm32g0xx_hal::{
    prelude::OutputPin,
    gpio::{PB6, PB7, Output, OpenDrain},
};

pub struct DecoderState {
    speed: u8,
    direction: bool,
    reverse_gate: bool, // used to prevent MM1 reverse spamming
    f0: bool,
    f0_fw: Option<PB6<Output<OpenDrain>>>,
    f0_rv: Option<PB7<Output<OpenDrain>>>,
}

impl DecoderState {

    pub const fn new() -> Self {
        Self {
            speed: 0,
            direction: true,
            reverse_gate: false, // TODO: this requires a speed packet to be sent before reverse can happen.
            f0: false,
            f0_fw: None,
            f0_rv: None,
        }
    }


    // Pass in the controlled GPIO at runtime
    pub fn init(&mut self, f0_fw: PB6<Output<OpenDrain>>, f0_rv: PB7<Output<OpenDrain>>) {
        self.f0_fw = Some(f0_fw);
        self.f0_rv = Some(f0_rv);
    }

    /// Updates the internal speed setting. Reports true if the setting has changed, otherwise false.
    pub fn update_speed(&mut self, speed: u8) -> bool {
        // update reverse gate
        self.reverse_gate = true;
        // update speed setting and report true if changed
        if self.speed != speed {
            self.speed = speed;
            true
        } else {
            false
        }
    }

    /// Updates direction from absolute direction. Also updates F0.
    pub fn update_direction(&mut self, direction: bool) -> bool {
        if self.direction != direction {
            self.direction = direction;
            self.apply_f0();
            true
        } else {
            false
        }
    }

    /// Updates direction from reverse instruction.
    pub fn update_reverse(&mut self) -> Option<bool> {
        // only update if reverse gate is set - this is done by update_speed();
        if self.reverse_gate {
            self.reverse_gate = false;
            self.direction = !self.direction;
            self.apply_f0();
            Some(self.direction)
        } else {
            None
        }
    }

    /// Update F0
    pub fn update_f0(&mut self, f0: bool) {
        if self.f0 != f0 {
            self.f0 = f0;
            self.apply_f0();
        }
    }

    /// Set the F0 outputs based on the current status of F0 and direction.
    /// This is run any time a change in direction or F0 status is found.
    fn apply_f0(&mut self) {
        let f0_fw = unsafe { self.f0_fw.as_mut().unwrap_unchecked() };
        let f0_rv = unsafe { self.f0_rv.as_mut().unwrap_unchecked() };
        match (self.direction, self.f0) {
            (false, true) => {
                f0_fw.set_high(); // turn off (open-drain)
                f0_rv.set_low(); // turn on (open-drain)
            }
            (true, true) => {
                f0_fw.set_low(); // turn on (open-drain)
                f0_rv.set_high(); // turn off (open-drain)
            }
            _ => {
                f0_fw.set_high(); // turn both off
                f0_rv.set_high();
            }
        }
    }

}