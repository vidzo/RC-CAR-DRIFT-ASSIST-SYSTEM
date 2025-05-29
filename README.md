Based on the great work by sidtalia with a lot of AI assistance on my side;

3 x Modes cylced by button on pin 5;

Rate mode - Classic yaw control - Cycled by pressing mode button, signalled by two short beeps

Angle mode - Almost like a heading hold mode, resets angle when steering input returns to neutral, Cycled by pressing mode button, signalled by one long beep

Yaw angle mode - Adaptive, stabilizes steering, selected by holding down mode button, signalled by three short beeps.

Each mode can be tuned to strength, 0.0 = full strength, 1.0 = no assiatnce. Button on pin 6 increases strengtgh in steps 0.1 with a flash of LED correlating to the srength step with max 
step using a long LED flash. Steps will return to the minimum number step if button is pressed again at max step 

_____________________

PID tuning available for each mode, with changes saved to respective profiles. 

Angle and Adaptive P, I, D, tuneable

Rate D tuneable. 

PID tuning button on pin 8 allows to cycle through P, I and D with long press, P will sound one beep, I sounds two beeps, D sounds three. SHort button press will increase by steps, signalled with a LED flash and a beep. Max number signalled with a long beep and will return to minimum number if pressed again from max.

All changes saved to rspective PID profiles and are presistent on next session.

_____________________

Includes a filter for gyro

Calibration on start up, complete after three flashes of LED, beep will signify which mode it's in. Mode is remebered from last session. 

Note;On first boot, assist will only work when PID numbers are added 

_____________________

- Hardware 
Arduino mini pro 3.3V 8MHz
GY-521 (inverted)
3 x tactile momentary buttons
1 x Buzzer (tested with passive)

- Setup
  Steering Input = A0
  Servo Output = 4
  Buttons = 5, 6, 8
  Buzzer = 7






# DRIFT-ASSIST-SYSTEM
gyro based stability assist system. Uses an MPU6050(use my copied version of mpu6050 library) and an arduino mini pro and some jumper wires.

1)Preferred microcontroller: arduino mini pro 5V 16MHz
2)Preferred IMU : mpu6050
3)Orientation of mpu6050: Z axis facing vertically upwards

SETUP: 

1)connect steering channel from receiver to pin 8

2)connect throttle channel from receiver to pin 9

3)connect steering servo to pin 3

4)connect throttle servo/esc to pin 4

5)SDA of mpu to A4, SCL of mpu to A5 

Upon startup, give it a few seconds before giving in any inputs. It is using those few seconds to caliberate the gyro. 
The controller takes partial control of the throttle and the steering to prevent the car from oversteering too much. The amount of "help"
being given by the controller can be changed by reducing the Kd value and by multiplying "mod(G[2])" by some number smaller than 1.
