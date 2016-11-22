# Controllers detail

## srh_mixed_position_velocity_controller

### Overview 

This controller consist of two nested loops. A position PID controller, closing the loop with the current position of the joint, sends velocity requests to a velocity controller
The velocity PID controller closes the loop with the current measured velocity of the joint, and sends effort commands to the joint interface.

On the shadow hand, the effort is forwarded via the driver to the motor boards.

### Specificities

#### Deadbands

override_to_current_effort parameter is required when a joint moves has a lot of backlash, and shows high variation in the friction profile within the motor/tendon system.

When override_to_current_effort is true, the current effort when entering the deadband is used as the force to maintain when in the deadband,
 instead of the current command when entering the deadband. 
 
Joint axis that suffer from gravity (like wrist joints), might not want to use this option, as it produces some jerk when entering the deadband under external forces.

default value is false


#### Min force threshold

motor_min_force_threshold parameter is used to not command the effort to the joint, if below a certain value. This value should match the threshold for backlash compensation detection
Indeed, it makes to sense to command an effort of 20 units if the backlash compensation is trigger the motor fullspeed until 40 units are reached (to wind back in the backlash zone as quickly as possible)

If backlash compensation is deactivated, the motor_min_force_threshold should be zero.


