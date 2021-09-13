***************************
Motor Alignment Calibration
***************************

When enabling a motor for the first time after powering the board, an alignment
calibration is automatically started to find the offset between encoder and
rotor.  This is needed for accurate field oriented control.

At the beginning of this process, the motor may move a little bit and will then
be held in place for a few seconds.  To achieve the best possible accuracy, make
sure the motors are not blocked (e.g. because they are pushing against an
obstacle) and avoid any external torque on the motor, e.g. move the robot into a
position where gravitational pull on the joints is minimal before starting the
system.  Also do not manually move the motors while the calibration is running.


**What happens when the calibration is inaccurate?**

The better the calibration, the higher the efficiency of the motor.  So with an
inaccurate calibration (assuming it is not completely off) it is still
possible to control the motor but a higher current needs to be applied to
achieve the same torque.
