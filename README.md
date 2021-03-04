# Arduino_motion_controller

/* CONTROL EXPOSURE AND CAMERA MOVEMENT WITH ARDUINO (UNO or NANO)

 *  This program drives 2 stepper motors; both are controlled with A4988 stepper motor controllers, a 0.96' OLED display and a rotary encoder with push button
 
 * Motor 1. for linear motion on a camera slider; 
 1cm diameter spindle, 200 steps per revolution and microstepping set to 1/4 (=800 steps per revol)
 Note: the program initializes by moving the motor 1 toward the end of the rail until activating a limit switch, then moves out until the limit switch is inactive again. This is position 0 and the step count starts for there.
 In my current implementation I use a 80cm rail, with ~70cm of useful length thus a value of 10500 steps of MaxSliderPosition  (15 steps ~ 0.8-1mm)
 the user defines the speed (default = 15 steps per cycle), and the program automatically calculates the speed decay to stop in 48 images (=2 sec of rendered video).
 
 * Motor 2. for panning (rotational movement). 200 steps per revol. 1/8 microstepping (=1.8/8= 0.225 degrees per step)
 *   the user defines 
 *    when to start panning (default = after sliding for 10cm), 
 *    how fast(number of steps per cycle; default = 1step), 
 *    how much (start to end angle of rotation default= 30 degrees from the start position), 
 *    as well as the direction (default= clockwise).
 Note: the start position is the current position when the program has initialized and the program counts from there.

 The controller can also be connected to a DSLR camera to control the exposure like an intervalometer.
   at every cycle of the user-defined interval, the camera waits for motors to execute movements before starting an exposure
   
**NEWLLY Implemented:** 
a panorama mode to make high resolution N-photo panoramas over a user defined angle 'lambda' at every step of the linear motion. (can be used for HDR too by setting a rotation of 0 degrees); 

Changed the LCD menu navigation to be more responsive to rotary encoder navigation.
The aspect could be much prettier, but the available space on the Arduino Uno is quite limited.

**NEEDED IMPROVEMENTS:**
1. Speed
For now the code continually runs trough a loop and checks if the active menu setting has changed and applies the according instructions from each level (regarding how to respond to rotary encoder rotation or clicking).
The large number of menus and the relatively limited internal clock speed from the Arduino Uno causes the responsivenes of the motor to be laggy in manual mode, and in general at each actuation of the photo trigger or of the motors, the loop is 'waiting' for current actions to finish before resuming the menu monitoring.
Being new to C++ based coding from the Arduino, I think that the code can largely be improved, but ultimately,even with pseudo loops baked into the code, the control unit should be upgraded from Arduino Uno to a unit with a'faster' internal clock (and more memory!), for the general reaction speed to be improved.

**Feel free to propose improvements to the code**

2. Star tracking:
I want to add pseudo-loops to perform a continuous rotational movement with a geared DC motor (at 15 degrees per hour) to offset earth's rotation, while exposing long shots

  
 * credits : For camera release I modified the pro-timer free arduino version from Gunther Wegner,  to take advantage of the exposure ramping 
  http://gwegner.de
  https://github.com/gwegner/LRTimelapse-Pro-Timer-Free

  For the rotary encoder I used Ben Bruxton's rotary.h library
  http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html
   
*/
