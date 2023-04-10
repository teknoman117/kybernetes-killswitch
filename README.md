kybernetes-killswitch
=====================
Arduino sketch for the ATTINY85 that manages the ESC and Servo enable lines. Requires an active "arm" signal from an RC radio (e.g. holding the trigger), a software "arm" signal over I2C, and a periodic heartbeat over I2C. Otherwise, the ESC and Servos will not be powered on.

Allows for effective remote-kill of Kybernetes, even if the motion controller arduino has a serious fault.
