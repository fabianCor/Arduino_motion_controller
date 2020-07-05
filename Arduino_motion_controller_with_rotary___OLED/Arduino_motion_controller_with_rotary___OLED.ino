//Arduino_motion_controller
/* CONTROL EXPOSURE AND CAMERA MOVEMENT WITH ARDUINO
    This program drives 2 stepper motors;
   1. for linear motion on a camera slider
    (change MaxSliderPosition depending on your equipment and microstepping setting)
     the user defines, speed, and the program automatically calculates the speed decay to stop in 2 sec of final movie (=48 images).
   2. and the other one for panning.
    the user defines when to start panning, how fast(number of steps per cycle), and how much (start to end angle of rotation), as well as the direction.

   The controller can also be wired to the camera to control the exposure.
    at every cycle of the user-defined interval, the camera waits for motors to execute movements before starting the exposure

   ####Comming soon: a panorama mode to make high res 2-photo stiches at every step of the linear motion.

   credits : For camera release I modified the pro-timer free arduino version from Gunther Wegner,  to take advantage of the exposure ramping
  http://gwegner.de
  https://github.com/gwegner/LRTimelapse-Pro-Timer-Free

  For the rotary encoder I used Ben Bruxton's rotary.h library
  http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html


*/

#include <rotary.h>
#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>
#include <TimerOne.h>


//////////////////////////////////////////////////////////////////////////
//#######################################################################
//      DEFINITIONS
//#######################################################################

//####  stepper motors ##############

/*Pin Definitions change accordingly to avalability
  There are 2 motors now, 'slider' is for longitudinal motion
  and 'pan' is for rotational motion
  --improvements will be:
    set the jumpers on the driver to the same step resolution
    this way only one n value is needed
  -- also the decay at the end of the rail is approximate and needs a function
    to better implement a smooth Nstep decay at the end of the longitudinal movement.
  -- need to implement a "start" value for when the pan will start moving
  -- implement user interface with step values converted to cm for slider and degree for pan movements.
*/
/* communication ports */
int Index;
//const int sliderStepPIN = 7; //commented out to save a few bytes of storage
//const int sliderDirPIN = 6;
//const int panStepPIN = 5;
//const int panDirPIN = 4;
const int LimitSwitch = 11; // limit end switch "on" when rail at the end of track
/* Motor step resolution */

const int DEFAULTstepNumber = 15;
int sliderNsteps = DEFAULTstepNumber;
int panNsteps = 1;
byte sliderDir;
byte panDir = 1;
int panStart = 10; //(= 200 = 1cm  steps at 1/4 step resolution; and 4 seconds of redered movie)
int panAngl = 30;
int currPosition;
unsigned long MaxSliderPosition = 10500; // rail=70cm utiles, spindle =4cm perimeter , motor=200 steps per rev. *4 (microstepping setting)
int currPan = 0;
int ManualSteps = 0;
int NpanoFrames = 1;
int panoramaAngl = 30;
int Frame = 1;

// ######## create OLED object #######

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);


//  ########## ENCODER ###############

/*  DEFINING ROTARY ENCODER dependencies
   encoder options will be called by 'encoder-><name of the option(attributes)>'
  Rotary(Encoder Pin 1, Encoder Pin 2, Button Pin)
  The encoder object  is actually initialized in the 'setup'
*/

// Initialize the Rotary object
// Rotary(Encoder Pin 1, Encoder Pin 2, Button Pin)
Rotary r = Rotary(2, 3, 8);


//###### GENERAL interrupt and timer objects #####################

const int Onboard_LED = 13;

const float RELEASE_TIME_DEFAULT = 0.1;      // default shutter release time for camera
const float MIN_DARK_TIME = 0.5;

const int decoupleTime = 50;      // time in milliseconds to wait before doing a single bulb exposure

float releaseTime = RELEASE_TIME_DEFAULT;             // Shutter release time for camera
unsigned long previousMillis = 0;   // Timestamp of last shutter release
unsigned long runningTime = 0;
unsigned long lastCheckTime = 0;

float interval = 15.0;          // the current interval
long maxNoOfShots = 0;
int isRunning = 0;            // flag indicates intervalometer is running
unsigned long bulbReleasedAt = 0;

int imageCount = 0;                     // Image count since start of intervalometer
/*Ramping*/
/* Taking out the interval ramping function to save storage space
    this could still be implemented in the future with some house cleaning
    of unnecessary variables (in particular char type)
    and by simplifying OLED display commands and the Menu actions
*/
unsigned long rampDuration = 10;    // ramping duration
float rampTo = 0.0;           // ramping interval
unsigned long rampingStartTime = 0;   // ramping start time
unsigned long rampingEndTime = 0;   // ramping end time
float intervalBeforeRamping = 0;    // interval before ramping

const float cMinInterval = 0.2;
const float cMaxInterval = 999;  // no intervals longer as 999secs - those would scramble the display

// K.H: EPROM Params
//EEPParams EEProm;


// HV Timer Interrupt Definitions
const byte shooting = 1;
const byte notshooting = 0;
byte cam_Release = notshooting;
int exposureTime = 0;    // Time for exposure in Timer Interrupt (10 msec)

//####### CAPTIONS AND MENUS ##########
/* This section defines the consecutive menus
    the text captions to display at various steps ect
*/
int maxNumber = 15;  //counting starts at 0
char* myMenus[] = {
  "SCR_INTERVAL",
  "SCR_MODE",
  "SCR_SHOTS",
  "SCR_EXPOSURE",
  "SCR_MOTOR1",
  "SCR_MOTOR2",
  "SCR_RAMPING",
  "SCR_RUNNING",
  "SCR_PAUSE",
  "SCR_DONE",
  "PowerSave",
  "SCR_SINGLE",
  "SCR_MANUAL_LONG",
  "SCR_MANUAL_ROT",
  "SCR_PANORAMA",
};

unsigned const int SCR_INTERVAL = 0;
unsigned const int SCR_MODE = 1;
unsigned const int SCR_SHOTS = 2;
unsigned const int SCR_EXPOSURE = 3;
unsigned const int SCR_MOTOR1 = 4;
unsigned const int SCR_MOTOR1_DIR = 41;
unsigned const int SCR_MOTOR2 = 5;
unsigned const int SCR_MOTOR2_Start = 52;
unsigned const int SCR_MOTOR2_Angl = 53;
unsigned const int SCR_MOTOR2_dir = 51;
unsigned const int SCR_RAMPING = 6;
unsigned const int SCR_RAMP_TO = 61;
unsigned const int SCR_RUNNING = 7;
unsigned const int SCR_PAUSE = 8;
unsigned const int SCR_DONE = 9;
unsigned const int PowerSave = 10;
unsigned const int powerSaveOn = 101;
unsigned const int  SCR_SINGLE = 11;
unsigned const int SCR_MANUAL_LONG = 12;
unsigned const int SCR_MANUAL_ROT = 13;
unsigned const int SCR_PANORAMA = 14;
unsigned const int SCR_PANO_OFFSET = 141;

unsigned const int MainScreen = 222;
const int MODE_M = 0;
const int MODE_BULB = 1;
int mode = MODE_M;

unsigned int x = 0;
unsigned int contrast = 10;
unsigned int currentMenu = MainScreen;

//////////////////////////////////////////////////////////////////////////
//#######################################################################
//           SETUP
//#######################################################################


void setup(void)
{
  Serial.begin(250000);

  pinMode(Onboard_LED, OUTPUT);
  digitalWrite(Onboard_LED, LOW); //turn off on-board LED

  /* Start OLED display and print welcome messages */
  u8x8.begin();
  u8x8.setFont(u8x8_font_8x13B_1x2_r);

  u8x8.clear (); // go home
  u8x8.setCursor(0, 0);

  u8x8.print("Fab's Timer 3.0");
  delay(2000);

  /* Initialize


    // printIntervalMenu();
    /* Motor initialization*/
  pinMode(7, OUTPUT); //Step for longitudinal motion
  pinMode(6, OUTPUT); //Direction for longitudinsal motion
  pinMode(5, OUTPUT); //Step for rotational motion
  pinMode(4, OUTPUT); //Direction for rotational motion
  pinMode(LimitSwitch, INPUT_PULLUP ); //Read signal from limit (Switch switch =1 when contact, PULLUP mode inverts this so that no contact =1)
  pinMode(12, OUTPUT);          // initialize output pin for camera release

  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.print("Initializing");
  u8x8.setCursor(0, 2);
  u8x8.print("Slider motor");
  u8x8.setCursor(0, 4);
  u8x8.print("Waiting for");
  u8x8.setCursor(0, 6);
  u8x8.print("limit switch...");

  // Start Homing procedure of Stepper Motor at startup
  digitalWrite(10, HIGH); //send power to limit switch
  while (digitalRead(LimitSwitch)) {  // Do this until the switch is activated
    digitalWrite(6, HIGH);      // (HIGH = anti-clockwise / LOW = clockwise)
    digitalWrite(7, HIGH);
    delayMicroseconds(1200);                       // Delay to slow down speed of Stepper
    digitalWrite(7, LOW);
    delayMicroseconds(1200);
  }

  while (!digitalRead(LimitSwitch)) { // Do this until the switch is not activated
    digitalWrite(6, LOW);
    digitalWrite(7, HIGH);
    delayMicroseconds(10000);                       // More delay to slow even more while moving away from switch
    digitalWrite(7, LOW);
    delayMicroseconds(10000);
  }


  currPosition = 0; // Reset position variable to zero
  sliderDir = 0;
  if (sliderDir == 1) {
    digitalWrite(6, HIGH);
  } else {
    digitalWrite(6, LOW);
  }
  u8x8.clear();

  /*Comment out*/
  digitalWrite(10, LOW); //cut power to limit switch

  //#################  START SCREEN ################
  u8x8.clear();
  u8x8.setCursor(0, 2);
  u8x8.print("Motor ready");
  u8x8.drawString(0, 4, "Choose");
  u8x8.drawString(0, 6, "Settings");
  delay(1200);


  blinkOLED();
  /*upper line*/
  u8x8.clear();
  pre(x);
  currentMenu == MainScreen;
}
///////////////////////////////////////////////////////////////////////////
//#######################################################################
//           MAIN LOOP
//#######################################################################


void loop(void)
{
  if (millis() > lastCheckTime + 100) {
    lastCheckTime = millis();

    if ( mode == MODE_BULB ) {
      possiblyEndLongExposure(); // checks regularly if a long exposure has to be cancelled
    }

    if ( currentMenu ==  SCR_SINGLE) {
      possiblyEndLongExposure();
    }

  }


  if (currentMenu == MainScreen) {
    volatile unsigned char turn = r.process();
    // read encoder value
    if (turn) {
      turn == DIR_CW ? x = x - 1 : x = x + 1;
      if (x < 0) {             // no values < 0; later: use unsigned int
        x = 13;    // roll over
        blinkOLED();
      }
      if (x > maxNumber - 1) {           // no more strings
        x = 0;                 // roll over;
        blinkOLED();
      }
      //upper line
      pre(x);
    }

    if (r.buttonPressedReleased(20)) {
      switch ( x ) {
        case SCR_INTERVAL:
          MenuString(x);
          printIntervalMenu();
          currentMenu = SCR_INTERVAL; // move to screen 'interval'
          break;
        case SCR_MODE:
          MenuString(x);
          printModeMenu();
          currentMenu = SCR_MODE;
          break;
        case SCR_SHOTS:
          MenuString(x);
          printNoOfShotsMenu();
          currentMenu = SCR_SHOTS;
          break;
        case SCR_EXPOSURE:
          MenuString(x);
          printExposureMenu();
          currentMenu = SCR_EXPOSURE;
          break;
        case SCR_MOTOR1:
          MenuString(x);
          printMotor1Menu();
          currentMenu = SCR_MOTOR1;
          break;
        case SCR_MOTOR2:
          currentMenu = SCR_MOTOR2;
          printMotor2Menu();
          break;
        case SCR_RAMPING:
          MenuString(x);
          currentMenu = SCR_RAMPING;
          break;
        case SCR_RUNNING:
          u8x8.clear();
          currentMenu = SCR_RUNNING;
          break;
        case SCR_PAUSE:
          isRunning = 0;
          u8x8.setCursor(0, 4);
          u8x8.print("Paused...");
          MenuString(x);
          currentMenu = SCR_PAUSE;
          break;
        case SCR_SINGLE:
          printSingleScreen();
          currentMenu = SCR_SINGLE;
          break;
        case SCR_DONE:
          printDoneScreen();
          currentMenu = SCR_DONE;
          break;
        case PowerSave:
          printPowerSaveMenu();
          currentMenu = PowerSave;
          break;
        case SCR_MANUAL_ROT:
          MenuString(x);
          u8x8.setCursor(0, 4);
          u8x8.print("Manual Rotation");
          currentMenu = SCR_MANUAL_ROT;
          ManualSteps = 0;
          break;
        case SCR_MANUAL_LONG:
          MenuString(x);
          u8x8.setCursor(0, 4);
          u8x8.print("Manual movt.");
          currentMenu = SCR_MANUAL_LONG;
          ManualSteps = 0;
          break;
        case SCR_PANORAMA:
          MenuString(x);
          printPanoramaMenu();
          currentMenu = SCR_PANORAMA;
          break;
      }
    }
  }

  if (currentMenu < 222) {
    volatile unsigned char encodeValue = r.process();
    if (encodeValue) {
      switch ( currentMenu ) {
        case SCR_INTERVAL:
          encodeValue == DIR_CCW ? interval = interval - 0.2 : interval = interval + 0.2;
          printIntervalMenu();
          break;
        case SCR_MODE:
          if ( mode == MODE_M ) {
            mode = MODE_BULB;
            u8x8.clearLine(6);
            u8x8.clearLine(7);
            u8x8.setCursor(8, 6);
            u8x8.print("BULB");
          } else {
            mode = MODE_M;
            u8x8.clearLine(6);
            u8x8.clearLine(7);
            u8x8.setCursor(8, 6);
            u8x8.print("M");
            releaseTime = RELEASE_TIME_DEFAULT;   // when switching to M-Mode, set the shortest shutter release time.
          }
          break;
        case SCR_SHOTS:
          if ( abs(maxNoOfShots) >= 2500) {
            encodeValue == DIR_CCW ? maxNoOfShots = maxNoOfShots - 100 : maxNoOfShots = maxNoOfShots + 100;
          } else if ( abs(maxNoOfShots) >= 1000) {
            encodeValue == DIR_CCW ? maxNoOfShots = maxNoOfShots - 50 : maxNoOfShots = maxNoOfShots + 50;
            printNoOfShotsMenu();
          } else {
            //maxNoOfShots = editValue(maxNoOfShots, 10);
            encodeValue == DIR_CCW ? maxNoOfShots = maxNoOfShots - 10 : maxNoOfShots = maxNoOfShots + 10;
          }
          if (maxNoOfShots <= 0) {
            maxNoOfShots = 0;
          }
          if ( maxNoOfShots >= 9999 ) { // prevents screwing the ui
            maxNoOfShots = 9999;
          }
          printNoOfShotsMenu();
          break;
        case SCR_EXPOSURE:
          encodeValue == DIR_CW ? releaseTime = releaseTime = (float)((int)(releaseTime * 10) + 1) / 10 : releaseTime = releaseTime = (float)((int)(releaseTime * 10) - 1) / 10;
          u8x8.clearLine(4);
          u8x8.clearLine(5);
          u8x8.setCursor(11, 4);
          u8x8.print(releaseTime);
          break;
        case SCR_MOTOR1:
          encodeValue == DIR_CCW ? sliderNsteps = sliderNsteps - 1 : sliderNsteps = sliderNsteps + 1;
          if (sliderNsteps < 1) {
            sliderNsteps = 0;
          }
          u8x8.setCursor(10, 2);
          u8x8.print(sliderNsteps);
          break;
        case SCR_MOTOR1_DIR:
          if (sliderDir == 1) {
            sliderDir = 0;
          } else if (sliderDir == 0) {
            sliderDir = 1;
          }
          u8x8.setCursor(10, 4);
          u8x8.print(sliderDir);
          break;
        case SCR_MOTOR2:
          encodeValue == DIR_CCW ? panNsteps = panNsteps - 1 : panNsteps = panNsteps + 1;
          if (panNsteps < 1) {
            panNsteps = 0;
          } else {
            panNsteps = panNsteps;
          }
          printMotor2Menu();
          break;
        case SCR_MOTOR2_dir:
          if (panDir == 1) {
            panDir = 0;
          } else if (panDir == 0) {
            panDir = 1;
          }
          printMotor2Menu();
          break;
        case SCR_MOTOR2_Start:
          encodeValue == DIR_CCW ? panStart = panStart - 1 : panStart = panStart + 1;
          printMotor2Menu();
          break;
        case SCR_MOTOR2_Angl:
          encodeValue == DIR_CCW ? panAngl = panAngl - 1 : panAngl = panAngl + 1;
          printMotor2Menu();
          break;
        case SCR_RAMPING:
          encodeValue == DIR_CCW ? rampDuration -= 10 : rampDuration += 10;
          printRampingMenu();
          break;
        case SCR_RAMP_TO:
          encodeValue == DIR_CCW ? rampTo = ((float)((int)(rampTo * 10) - 1) / 10) : ((float)((int)(rampTo * 10) + 1) / 10);
          printRampingMenu();
          break;
        case SCR_RUNNING:
          printRunningScreen();
          isRunning = 1;
          u8x8.setCursor(0, 6);
          u8x8.print("< Settings");
          break;
        case SCR_SINGLE:
          if (releaseTime < 60) {
            encodeValue == DIR_CCW ? releaseTime = (float)((int)(releaseTime - 1)) : releaseTime = (float)((int)(releaseTime + 1)) ;
            if (releaseTime < RELEASE_TIME_DEFAULT )  // if it's too short after decrementing, set to the default release time.
              releaseTime = RELEASE_TIME_DEFAULT;
            printSingleScreen();
          } else {
            encodeValue == DIR_CCW ? releaseTime = (float)((int)(releaseTime - 10)) : releaseTime = (float)((int)(releaseTime + 10));
            printSingleScreen();
          }
          break;
        case SCR_DONE:
          currentMenu = MainScreen;
          pre(x);
          break;
        case PowerSave:
          u8x8.setPowerSave(true);
          currentMenu = powerSaveOn;
          break;
        case powerSaveOn:
          u8x8.setPowerSave(false);
          currentMenu = SCR_RUNNING;
          break;
        case SCR_MANUAL_LONG:
          if (encodeValue == DIR_CW ) {
            ManualSteps += 200 ;
            digitalWrite(6, HIGH);
            int target = currPosition + ManualSteps;
            Serial.println(ManualSteps);
            if ( target >= MaxSliderPosition) {
              target = MaxSliderPosition;
              u8x8.clear();
              u8x8.setCursor(0, 4);
              u8x8.print("Max pos. reached");
            }
            while ( currPosition < target )
            {
              Serial.print(".");
              digitalWrite(7, HIGH);
              delayMicroseconds(1200); // 1000000 * stepper_1DelayTime/n = time between steps in microseconds
              digitalWrite(7, LOW);
              delayMicroseconds(1200);
              currPosition++;
            }
          } else if (encodeValue == DIR_CCW) {
            digitalWrite(6, LOW);
            ManualSteps = ManualSteps + 200;
            int target = currPosition - ManualSteps;
            if ( target <= 0) {
              target = 0;
              u8x8.clear();
              u8x8.setCursor(0, 4);
              u8x8.print("Max pos. reached");
            }
            while ( currPosition > target )
            {
              digitalWrite(7, HIGH);
              delayMicroseconds(1200); // 1000000 * stepper_1DelayTime/n = time between steps in microseconds
              digitalWrite(7, LOW);
              delayMicroseconds(1200);
              currPosition--;
            }
          }
          break;
        case SCR_MANUAL_ROT:
          if (encodeValue == DIR_CW ) {
            ManualSteps += 20 ;
            int panTarget = currPan + ManualSteps;
            digitalWrite(4, HIGH);
            while (currPan < panTarget)
            {
              digitalWrite(5, HIGH);
              delayMicroseconds(1200); // 1000000 * stepper_1DelayTime/n = time between steps in microseconds
              digitalWrite(5, LOW);
              delayMicroseconds(1200);
              currPan++;
            }
          } else {
            ManualSteps += 20 ;
            int panTarget = currPan - ManualSteps;
            digitalWrite(4, LOW);
            while (currPan > panTarget)
            {
              digitalWrite(5, HIGH);
              delayMicroseconds(1200); // 1000000 * stepper_1DelayTime/n = time between steps in microseconds
              digitalWrite(5, LOW);
              delayMicroseconds(1200);
              currPan--;
            }
          }
          break;
        case SCR_PANORAMA:
          NpanoFrames += 1;
          break;
        case SCR_PANO_OFFSET:
          panoramaAngl += 1;
          break;
      }
    }// end of if (encodeValue)

    if (r.buttonPressedReleased(20)) {
      switch ( currentMenu ) {
        case SCR_INTERVAL:
          currentMenu = MainScreen;
          x = x + 1;
          pre(x);
          break;
        case SCR_MODE:
          currentMenu = MainScreen;
          x = x + 1;
          pre(x);
          break;
        case SCR_SHOTS:
          currentMenu = MainScreen;
          x = x + 1;
          pre(x);
          break;
        case SCR_EXPOSURE:
          currentMenu = MainScreen;
          x = x + 1;
          pre(x);
          break;
        case SCR_MOTOR1:
          currentMenu = SCR_MOTOR1_DIR;
          break;
        case SCR_MOTOR1_DIR:
          currentMenu = MainScreen;
          x = x + 1;
          pre(x);
          break;
        case SCR_MOTOR2:
          currentMenu = SCR_MOTOR2_dir;
          printMotor2Menu();
          break;
        case SCR_MOTOR2_dir:
          currentMenu = SCR_MOTOR2_Start;
          printMotor2Menu();
          break;
        case SCR_MOTOR2_Start:
          currentMenu = SCR_MOTOR2_Angl;
          printMotor2Menu();
          break;
        case SCR_MOTOR2_Angl:
          currentMenu = MainScreen;
          x = x + 1;
          pre(x);
          break;
        case SCR_RAMPING:
          currentMenu = SCR_RAMP_TO;
          break;
        case SCR_RAMP_TO:
          currentMenu = MainScreen;
          x = x + 1;
          pre(x);
          break;
        case SCR_RUNNING:
          currentMenu = MainScreen;
          x = x + 1;
          pre(x);
          break;
        case SCR_PAUSE:
          u8x8.clear();
          currentMenu = SCR_RUNNING;
          x = currentMenu;
          isRunning = 1;
          previousMillis = millis() - (imageCount * 1000); // prevent counting the paused time as running time;
          break;
        case SCR_SINGLE:
          u8x8.clear();
          u8x8.setCursor(0, 2);
          u8x8.print("Decoupling...");
          delay( decoupleTime );
          u8x8.clear();
          releaseCamera();
          u8x8.clear();
          printSingleScreen();
          break;
        case SCR_DONE:
          stopShooting();
          currentMenu = MainScreen;
          pre(x);
          break;
        case powerSaveOn:
          u8x8.setPowerSave(false);
          currentMenu = MainScreen;
          x = currentMenu;
          pre(x);
          break;
        case SCR_MANUAL_ROT:
          currentMenu = MainScreen;
          x = currentMenu;
          pre(x);
          break;
        case SCR_MANUAL_LONG:
          currentMenu = MainScreen;
          x = currentMenu;
          pre(x);
          break;
        case SCR_PANORAMA:
          currentMenu = SCR_PANO_OFFSET;
          printPanoramaMenu();
          break;
        case SCR_PANO_OFFSET:
          currentMenu = MainScreen;
          x = currentMenu;
          pre(x);
          break;
      }
    }
  }

  /* Interrupt camera release*/
  if (cam_Release == shooting)
  {

    if (exposureTime == 0)       // End of exposure
    {
      cam_Release = notshooting;
      digitalWrite(12, LOW);

      if ( currentMenu == SCR_RUNNING ) { // clear exposure indicator
        u8x8.setCursor(15, 6);
        u8x8.noInverse();
        u8x8.print(" ");
      }
    }
  }
  if ( isRunning ) {  // release camera, do ramping if running
    running();
  }
}



///////////////////////////////////////////////////////////////////////////
//#######################################################################
//           MAIN LOOP END
//#######################################################################


void MenuString(int x) {    // screen title
  u8x8.clear();
  u8x8.print(myMenus[x]);
  u8x8.noInverse();
}

void pre(int x)  // menu scrolling
{
  u8x8.clear();

  u8x8.setCursor(1, 2);
  u8x8.print(myMenus[x + 1]);
  u8x8.setCursor(1, 4);
  u8x8.print(myMenus[x + 2]);
  u8x8.setCursor(1, 6);
  u8x8.print(myMenus[x + 3]);
  u8x8.inverse();
  u8x8.setCursor(0, 0);
  u8x8.print(myMenus[x]);
  u8x8.noInverse();
}

void stopShooting() {    //stop shooting
  isRunning = 0;
  imageCount = 0;
  runningTime = 0;
  bulbReleasedAt = 0;
}

void running() {        // shooting

  // do this every interval only
  if ( ( millis() - previousMillis ) >=  ( ( interval * 1000 )) ) {

    if ( ( maxNoOfShots != 0 ) && ( imageCount >= maxNoOfShots ) ) { // sequence is finished
      // stop shooting
      isRunning = 0;
      currentMenu = SCR_DONE;
      u8x8.clear();
      printDoneScreen(); // invoke manually
      stopShooting();

    } else { // is running
      runningTime += (millis() - previousMillis );
      previousMillis = millis();
      releaseCamera();
      imageCount++;
    }
  }

  // do this always (multiple times per interval)
  possiblyRampInterval();
}

/**
   Actually release the camera
*/
void releaseCamera() {

  /*Running the stepper motor here at the end of the exposure*/
  int DecayLength = decay2sec(sliderNsteps);
  MaxSliderPosition = MaxSliderPosition - DecayLength;
  // running the stepper motor
  if (sliderDir == 1) {
    digitalWrite(6, HIGH);
  } else {
    digitalWrite(6, LOW);
  }
  if (currPosition <= MaxSliderPosition) {
    for (Index = 0; Index < sliderNsteps ; Index++)
    {
      digitalWrite(7, HIGH);
      delayMicroseconds(1200); // 1000000 * stepper_1DelayTime/n = time between steps in microseconds
      digitalWrite(7, LOW);
      delayMicroseconds(1200);
      currPosition++;
    }
  } else {
    /*Setting a speed decay at the end of the rail over 48 shots*/
    int stepchg = sliderNsteps / 48;
    int totchg = stepchg;
    sliderNsteps = round(sliderNsteps - totchg);
    for (Index = 0; Index < sliderNsteps ; Index++) //reducing Nsteps by 10% at every loop
    {
      digitalWrite(7, HIGH);
      delayMicroseconds(1200); // 1000000 * stepper_1DelayTime/n = time between steps in microseconds
      digitalWrite(7, LOW);
      delayMicroseconds(1200);
      currPosition++;
    } // end of stepper loop
    totchg = totchg + stepchg;
  }
  delay(10);// 100 micro-seconds delay then trigger camera

  // move the pan motor
  if (panDir = 1) {
    digitalWrite(4, HIGH);
  } else {
    digitalWrite(4, LOW);
  }

  if ((currPosition >= panStart * 400) && (currPan <= panAngl / (1.8 / 8))) {
    for (Index = 0; Index < panNsteps ; Index++)
    {
      digitalWrite(5, HIGH);
      delayMicroseconds(1200); // 1000000 * stepper_1DelayTime/n = time between steps in microseconds
      digitalWrite(5, LOW);
      delayMicroseconds(1200);
      currPan++;
    }
  }
  //########################################################

  /*Panorama mode */
  if (NpanoFrames > 1) {
    //if the panorama mode is active
    if (Frame < NpanoFrames) {
      if (panDir = 1) {
        // normal direction
        digitalWrite(4, HIGH);
      } else {
        digitalWrite(4, LOW);
      }
      for (Index = 0; Index < panoramaAngl / (1.8 / 8) ; Index++)
      {
        digitalWrite(5, HIGH);
        delayMicroseconds(1200); // 1000000 * stepper_1DelayTime/n = time between steps in microseconds
        digitalWrite(5, LOW);
        delayMicroseconds(1200);
      }
    }
    if (Frame = NpanoFrames) {
      if (panDir = 1) {
        // inverting direction by setting pin 4 to the opposite state
        digitalWrite(4, LOW);
      } else {
        digitalWrite(4, HIGH);
      }
      for (Index = 0; Index < (panoramaAngl * NpanoFrames) / (1.8 / 8) ; Index++)
        //come back to Frame 1 position
      {
        digitalWrite(5, HIGH);
        delayMicroseconds(1200); // 1000000 * stepper_1DelayTime/n = time between steps in microseconds
        digitalWrite(5, LOW);
        delayMicroseconds(1200);
      }
    }
    // count the frames until reaching the max number of panorama frames
    // then loop back to 1
    Frame += 1;
    if (Frame = NpanoFrames) {
      Frame = 1;
    }
  }

  // short trigger in M-Mode
  if ( releaseTime < 1 ) {
    if ( currentMenu == SCR_RUNNING ) { // display exposure indicator on running screen only
      u8x8.setCursor(7, 1);
      u8x8.print((char)255); // print a black block
    }
    // HV changes für Interrupt Cam release and Display indicator handling
    if (releaseTime < 0.2) {
      exposureTime = releaseTime * 150;     // for better viewability
    }
    else {
      exposureTime = releaseTime * 100;
    }
    cam_Release = shooting;

    digitalWrite(12, HIGH);


  } else { // releaseTime > 1 sec

    // long trigger in Bulb-Mode for longer exposures
    if ( bulbReleasedAt == 0 ) {
      bulbReleasedAt = millis();
      // HV changes für Interrupt Cam release and Display indicator handling
      u8x8.setCursor(7, 1);
      u8x8.inverse();
      u8x8.print(" ");
      u8x8.noInverse();
      exposureTime = releaseTime * 100;
      cam_Release = shooting;
      digitalWrite(12, HIGH);

    }
  }
}

void possiblyEndLongExposure() {
  if ( ( bulbReleasedAt != 0 ) && ( millis() >= ( bulbReleasedAt + releaseTime * 1000 ) ) ) {
    bulbReleasedAt = 0;
    // HV changes für Interrupt Cam release and Display indicator handling
    digitalWrite(12, LOW);


    if ( currentMenu == SCR_SINGLE ) {
      printSingleScreen();
    }
  }
}

void possiblyRampInterval() {

  if ( ( millis() < rampingEndTime ) && ( millis() >= rampingStartTime ) ) {
    interval = intervalBeforeRamping + ( (float)( millis() - rampingStartTime ) / (float)( rampingEndTime - rampingStartTime ) * ( rampTo - intervalBeforeRamping ) );

    if ( releaseTime > interval - MIN_DARK_TIME ) { // if ramping makes the interval too short for the exposure time in bulb mode, adjust the exposure time
      releaseTime =  interval - MIN_DARK_TIME;
    }

  } else {
    rampingStartTime = 0;
    rampingEndTime = 0;
  }
}

/* /////////////////////////////////////////////////////////////////////
  //###############   MENU ACTIONS and display #############
  ////////////////////////////////////////////////////////////////////////
*/
void printIntervalMenu() {
  // display Interval between motor actions in seconds
  u8x8.setCursor(1, 2);
  u8x8.print("Interval (sec)");
  //u8x8.draw1x2String(1, 2, "Interval (sec)");
  u8x8.setCursor(8, 6);
  u8x8.print(interval);
}

void printModeMenu() {
  u8x8.clear();
  u8x8.setCursor(1, 4);
  u8x8.print("Mode :");
  u8x8.setCursor(8, 6);
  if (mode == 0) {
    u8x8.print("M");
  } else {
    u8x8.print("BULB");
  }

}

void printNoOfShotsMenu() {

  u8x8.clear();
  u8x8.setCursor(0, 2);
  u8x8.print("No of shots");
  u8x8.setCursor(2, 4);
  if ( maxNoOfShots > 0 ) {
    u8x8.print( printInt( maxNoOfShots, 4 ) );
  } else {
    u8x8.print( "unlimited" );
  }
  u8x8.print( "       "); // clear rest of display
}

void printExposureMenu() {

  u8x8.setCursor(0, 2);
  u8x8.print("release time : ");
  u8x8.setCursor(11, 4);
  u8x8.print(releaseTime);
}

void printMotor1Menu() {
  u8x8.clear();
  //  u8x8.setFont(u8x8_font_8x13B_1x2_r);
  u8x8.setCursor(0, 4);
  u8x8.print("Direction");

  u8x8.setCursor(0, 2);
  u8x8.print("N Steps");

  u8x8.setCursor(10, 4);
  u8x8.print(sliderDir);

  u8x8.setCursor(10, 2);
  u8x8.print(sliderNsteps);

}

void printMotor2Menu() {

  u8x8.clear();

  u8x8.setCursor(0, 0);
  if (currentMenu == 5) {
    u8x8.inverse();
    u8x8.print("N Steps :   ");
    u8x8.print(panNsteps);
    u8x8.noInverse();
  } else {
    u8x8.print("N Steps :   ");
    u8x8.print(panNsteps);
  }
  u8x8.setCursor(0, 2);
  if (currentMenu == 51) {
    u8x8.inverse();
    u8x8.print("Direction   ");
    u8x8.print(panDir);
    u8x8.noInverse();
  } else {
    u8x8.print("Direction   ");
    u8x8.print(panDir);
  }

  u8x8.setCursor(0, 4);
  if (currentMenu == 52) {
    u8x8.inverse();
    u8x8.print("Start (cm) ");
    u8x8.print(panStart);
    u8x8.noInverse();
  } else {
    u8x8.print("Start (cm) ");
    u8x8.print(panStart);
  }

  u8x8.setCursor(0, 6);
  if (currentMenu == 53) {
    u8x8.inverse();
    u8x8.print("Angl       ");
    u8x8.print(panAngl);
    u8x8.noInverse();
  } else {
    u8x8.print("Angl       ");
    u8x8.print(panAngl);
  }
}

void printRampingMenu() {

  u8x8.clear();

  u8x8.setCursor(0, 0);
  if (currentMenu == 6) {
    u8x8.inverse();
    u8x8.print("Ramp Time (min) ");
    u8x8.setCursor(0, 2);
    u8x8.print(rampDuration);
    u8x8.noInverse();
  } else {
    u8x8.print("Ramp Time (min) ");
    u8x8.setCursor(0, 2);
    u8x8.print(rampDuration);
  }
  u8x8.setCursor(0, 4);
  if (currentMenu == 61) {
    u8x8.inverse();
    u8x8.print("Ramp to (Intvl.)");
    u8x8.setCursor(0, 6);
    u8x8.print( printFloat( rampTo, 5, 1 ) );
    u8x8.noInverse();
  } else {
    u8x8.print("Ramp Time (min) ");
    u8x8.setCursor(0, 2);
    u8x8.print( printFloat( rampTo, 5, 1 ) );
  }
}

void printDoneScreen() {

  // print elapsed image count))
  u8x8.clear();

  u8x8.setCursor(0, 0);
  u8x8.print("Done ");
  u8x8.print( imageCount );
  u8x8.print( " shots");

  // print elapsed time when done
  u8x8.setCursor(0, 2);
  u8x8.print( "t=");
  u8x8.print( fillZero( runningTime / 1000 / 60 / 60 ) );
  u8x8.print( ":" );
  u8x8.print( fillZero( ( runningTime / 1000 / 60 ) % 60 ) );

  u8x8.setCursor(0, 4);
  u8x8.print("turn = go back");
  u8x8.setCursor(0, 6);
  u8x8.print("click = confirm");

}

void printRunningScreen() {


  u8x8.setCursor(0, 0);
  u8x8.print("RUNNING");
  u8x8.setCursor(0, 2);
  u8x8.print( printInt( imageCount, 4 ) );

  if ( maxNoOfShots > 0 ) {
    u8x8.setCursor(0, 4);
    u8x8.print( " R:" );
    u8x8.print( printInt( maxNoOfShots - imageCount, 4 ) );
    u8x8.print( " " );

    u8x8.setCursor(0, 6);
    // print remaining time
    unsigned long remainingSecs = (maxNoOfShots - imageCount) * interval;
    u8x8.print( "T-");
    u8x8.print( fillZero( remainingSecs / 60 / 60 ) );
    u8x8.print( ":" );
    u8x8.print( fillZero( ( remainingSecs / 60 ) % 60 ) );
  }

  updateTime();
}

void printSingleScreen() {
  u8x8.setCursor(0, 0);

  if ( releaseTime < 1 ) {
    u8x8.print( "Single Exposure");
    u8x8.setCursor(0, 2);
    u8x8.print(releaseTime); // under one second
    u8x8.print("       ");
  } else {
    u8x8.print( "Bulb Exposure  ");
    u8x8.setCursor(0, 2);

    if ( bulbReleasedAt == 0 ) { // if not shooting
      // display exposure time setting
      int hours = (int)releaseTime / 60 / 60;
      int minutes = ( (int)releaseTime / 60 ) % 60;
      int secs = ( (int)releaseTime ) % 60;
      String sHours = fillZero( hours );
      String sMinutes = fillZero( minutes );
      String sSecs = fillZero( secs );

      u8x8.print( sHours );
      u8x8.print(":");
      u8x8.print( sMinutes );
      u8x8.print( "\'");
      u8x8.print( sSecs );
      u8x8.print( "\"");
      u8x8.print( " " );
    }
  }

  if ( bulbReleasedAt == 0 ) { // currently not running

    u8x8.setCursor(0, 4);
    u8x8.print( "click to FIRE" );

  } else { // running
    unsigned long runningTime = ( bulbReleasedAt + releaseTime * 1000 ) - millis();

    int hours = runningTime / 1000 / 60 / 60;
    int minutes = ( runningTime / 1000 / 60 ) % 60;
    int secs = ( runningTime / 1000 ) % 60;

    String sHours = fillZero( hours );
    String sMinutes = fillZero( minutes );
    String sSecs = fillZero( secs );

    u8x8.setCursor(0, 2);
    u8x8.print("        "); // clear time setting display
    u8x8.setCursor(8, 2);
    u8x8.print( sHours );
    u8x8.setCursor(10, 2);
    u8x8.print(":");
    u8x8.setCursor(11, 2);
    u8x8.print( sMinutes );
    u8x8.setCursor(13, 2);
    u8x8.print(":");
    u8x8.setCursor(14, 2);
    u8x8.print( sSecs );
  }
}

void printPowerSaveMenu() {
  u8x8.clear();
  u8x8.setCursor(0, 2);
  u8x8.print("Enter pwr save");
  u8x8.setCursor(0, 4);
  u8x8.print("mode. ");
}

void printPanoramaMenu() {

  u8x8.clear();

  u8x8.setCursor(0, 0);
  if (currentMenu == 14) {
    u8x8.inverse();
    u8x8.print("N Frames :   ");
    u8x8.print(NpanoFrames);
    u8x8.noInverse();
  } else {
    u8x8.print("N Frames :   ");
    u8x8.print(NpanoFrames);
  }
  u8x8.setCursor(0, 2);
  if (currentMenu == 141) {
    u8x8.inverse();
    u8x8.print("Angle offset:  ");
    u8x8.print(panoramaAngl);
    u8x8.noInverse();
  } else {
    u8x8.print("Angle offset:  ");
    u8x8.print(panoramaAngl);
  }
}

  //###############GRAPHIC FUNCTIONS ###############


  void blinkOLED() {
    for (int i = 0; i < 3; i++)
    {
      u8x8.setContrast(1);
      //u8x8.sleepOn();
      delay(50);
      u8x8.setContrast(255);
      //u8x8.sleepOff();
      delay(50);
    }
  }

  void draw_bar(uint8_t c, uint8_t is_inverse)
  {

    uint8_t r;
    u8x8.setInverseFont(is_inverse);
    for ( r = 0; r < u8x8.getRows(); r++ )
    {
      u8x8.setCursor(c, r);
      u8x8.print(" ");
    }
  }

  /**
     Update the time display in the main screen
  */
  void updateTime() {

    unsigned long finerRunningTime = runningTime + (millis() - previousMillis);

    if ( isRunning ) {

      int hours = finerRunningTime / 1000 / 60 / 60;
      int minutes = (finerRunningTime / 1000 / 60) % 60;
      int secs = (finerRunningTime / 1000 ) % 60;

      String sHours = fillZero( hours );
      String sMinutes = fillZero( minutes );
      String sSecs = fillZero( secs );

      u8x8.setCursor(8, 2);
      u8x8.print( sHours );
      u8x8.setCursor(10, 2);
      u8x8.print(":");
      u8x8.setCursor(11, 2);
      u8x8.print( sMinutes );
      u8x8.setCursor(13, 2);
      u8x8.print(":");
      u8x8.setCursor(14, 2);
      u8x8.print( sSecs );
    } else {
      u8x8.setCursor(8, 2);
      u8x8.print("   Done!");
    }
  }

  /*decay2sec calculates how many steps it takes
    to gradually stop the motor
    over 48 icrements (=2 seconds at a 24fps frame rate in the rendered video)*/

  int decay2sec(int x) {

    int count = 1;
    int stepchg = x / 48;
    int totchg = stepchg;
    int y;
    while (count <= 48) {

      y += round(x - totchg);
      totchg += stepchg;
      count ++;
    }
    return count;
  }
  // ----------- HELPER METHODS -------------------------------------

  int editValue (int value , int increment) {
    volatile unsigned char turn = r.process();
    turn == DIR_CCW ? value = value - increment : value = value + increment;
    return value;
  }

  /**
     Fill in leading zero to numbers in order to always have 2 digits
  */
  String fillZero( int input ) {

    String sInput = String( input );
    if ( sInput.length() < 2 ) {
      sInput = "0";
      sInput.concat( String( input ));
    }
    return sInput;
  }

  String printFloat(float f, int total, int dec) {

    static char dtostrfbuffer[8];
    String s = dtostrf(f, total, dec, dtostrfbuffer);
    return s;
  }

  String printInt( int i, int total) {
    float f = i;
    static char dtostrfbuffer[8];
    String s = dtostrf(f, total, 0, dtostrfbuffer);
    return s;
  }
