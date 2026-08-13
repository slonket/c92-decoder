# c92-decoder

### TODO

- [ ] Write a comprehensive README.md
- [ ] Implement F1-F4 GPIO (held by decoder_state?).
    - Framework DONE - need F3/F4 GPIO setup.
- [ ] Make acceleration potentiometer non-linear and with better range in motor control.
- [ ] Add emergency stop and decoder disable for motor current limit trip.
- [ ] Add analog (DC) operation.
- [ ] Add marklin braking section detection.
- [ ] Store absolute direction in flash between power cycles for MM1.
    - [ ] Store complete state with F1-F4. Use brownout detection?
- [ ] Check MM 81-255 work on other controllers like MS2.
    - [ ] Check the 83-191 address swap on IB-Basic isn't just a bug.
    - [ ] Check all addresses work.
- [x] Connect F4 state to ramp bypass in motor control.
- [x] Add motorola old functions F1-F4.
- [x] Extend MM to use 255 addresses.

### Issues

- [x] The headlight F0 (decoder state) and motor drive direction can go out of sync on motorola old. this is likely due to the use of "change direction" rather than "absolute direction". It is hard to replicate this bug but the change likely requires update_reverse() to provide an absolute direction, and to not use "reverse" on motor_control. This keeps motor_state and motor_control in sync and lets motor_state exclusively handle detection of direction changes.
    - FIXED - Removed reverse in motor control. It is now absolute from decoder state.
- [x] Loco moves briefly in new direction after reverse on MM1.'
    - FIXED - Reversing would only change direction, but not update speed. Fixed by setting motor speed to 0 after update_reverse() for decoder state.

### Revisit

- [ ] Feedback control and PI control system tuning.
- [ ] Speed range minimum value from potentiometer.
    - [ ] Minimum (crawl) speed tuning.
- [ ] Accelation range from potentiometer.
- [ ] Lenz/MM pulse timing tolerances (notably MM between controllers).
- [ ] Initial GPIO states for functions and motor outputs.