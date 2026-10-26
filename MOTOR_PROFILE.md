# MCU S-curve velocity profile

UDP target and MotorStatus.target_velocity remain the final signed target.
MCU generates a quintic S curve for RPDO velocity on the existing 10 ms CAN
cycle. EPOS PVM mode 3 and its tuned PI loop track the trajectory.

Core/Inc/motor_profile.h owns the settings. MOTOR_SCURVE_RATE=300 rpm/s sets
peak acceleration for a transition starting at constant speed. Duration is
ceil(1.875 * abs(delta rpm) / rate * 1000), minimum 100 ms. Thus 0 to 150 rpm
and 150 to 0 take 938 ms; 0 to 50 takes 313 ms. The normalized curve is
10u^3-15u^4+6u^5, with zero acceleration and jerk at both ends. Keeping the old
500 ms duration would increase peak acceleration, so duration was extended.

Mid-ramp retargeting preserves velocity and acceleration but can change jerk;
it does not guarantee the same acceleration bound or monotonicity as a
constant-speed start. This is not a general constrained trajectory planner.
Duplicate commands do not restart the curve. Host tests cover both signs,
deceleration, retarget continuity, signed endpoints and timer wrap.

Driver 0x6083/0x6084 are 10000 rpm/s and 0x6086=0, so the earlier 300 rpm/s
linear driver ramp does not reshape the MCU trajectory at tested speeds.
The secondary driver ceiling may reshape higher-speed trajectories. Writes
are SDO-confirmed before enable; rejection prevents enable. Units assume the
commissioned rpm/rpm/s scaling. No Flash store or PI gain changes.

Reversal waits for the zero-target trajectory to finish AND fresh TPDO
standstill confirmation before the opposite trajectory starts. Quick Stop,
fault and lease stops bypass the S curve using the existing stop path and
unchanged 0x6085. Enable resets the trajectory; errors discard it. Review the
existing 5-second reversal timeout for large velocity changes.

Download firmware and reset drivers before testing. CSV drive_target reports
the shaped RPDO command, while GUI target remains the final goal. About
125 ms PC samples cannot establish mechanical jerk. This feature does not
provide stall or collision protection. Hardware motion remains untested.
