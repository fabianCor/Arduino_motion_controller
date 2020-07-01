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
   
####Comming soon: 
a panorama mode to make high resolution N-photo panoramas over a user defined angle 'lambda' at every step of the linear motion. (can be used for HDR to); 
Future implementations will attempt to follow earth's rotation to suppress star trailling over very long exposure times (>30 sec); but it will not fit in the 100Kb of an arduino nano chip.
  
 * credits : For camera release I modified the pro-timer free arduino version from Gunther Wegner,  to take advantage of the exposure ramping 
  http://gwegner.de
  https://github.com/gwegner/LRTimelapse-Pro-Timer-Free

  For the rotary encoder I used Ben Bruxton's rotary.h library
  http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html
   
*/
