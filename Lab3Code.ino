/* a closed loop proportional control
gkn 20230209
*/
// Include Libraries
// This is the PinChangeInterrupt package by NicoHood, available in the library search
#include <PinChangeInterrupt.h>
// Driver definitions
// If you have a kit with the moto shield, set this to true
// If you have the Dual H-Bridge controller w/o the shield, set to false
#define SHIELD true
//SHIELD Pin varables - cannot be changed
#define motorApwm 3
#define motorAdir 12
#define motorBpwm 11
#define motorBdir 13
//Driver Pin variable - any 4 analog pins (marked with ~ on your board)
#define IN1 9
#define IN2 10
#define IN3 5
#define IN4 6
// Lab Specific definitions
// Defining these allows us to use letters in place of binary when
// controlling our motors
#define A 0
#define B 1
#define pushButton 10  // install a Pullup button with its output into Pin 2
// You may want to define pins for buttons as bump sensors. Pay attention to other
used pins.
// The digital pins we'll use for the encoders
#define EncoderMotorLeft 8
#define EncoderMotorRight 7
  // Initialize encoder counts to 0
  // These are volatile because they change during interrupt functions
volatile unsigned int leftEncoderCount = 0;
volatile unsigned int rightEncoderCount = 0;

//These are to build your moves array, a la Lab 2
#define FORWARD 30.48
#define LEFT 1
#define RIGHT -1

// CONSTANTS TO MODIFY IN THIS LAB
// Drive constants - dependent on robot configuration
#define EncoderCountsPerRev 120.0
#define DistancePerRev 25.5  // TODO: Measure wheel diameter (8.1
cm) (Calculated Value 25.44)
#define DegreesPerRev 165  // TODO: Measure distance between wheels (18.8 cm)
(Calculated Value 155.58)

// Proportional Control constants
// what are your ratios of PWM:Encoder Count error?
#define kP_A 1.8
#define kP_B 2.0

// Derivative Control constants
#define kD_A 5.0
#define kD_B 1.0

// how many encoder counts from your goal are acceptable?
#define distTolerance 3
// Deadband power settings
// The min PWM required for your robot's wheels to still move
// May be different for each motor

#define minPWM_A 75
#define minPWM_B 75
#define maxPWM_A 202
#define maxPWM_B 230
// Lab specific variables (in cm)
// Fill in this array will forward distances and turn directions in the maze (a la Lab2)


int moves[] = {FORWARD, LEFT, FORWARD, LEFT, FORWARD, RIGHT, FORWARD*3, RIGHT,
FORWARD*2, RIGHT, FORWARD};
void setup() {
  // set stuff up
  Serial.begin(9600);
  motor_setup();
  pinMode(pushButton, INPUT_PULLUP);
  // add additional pinMode statements for any bump sensors
  // Attaching Wheel Encoder Interrupts
  Serial.print("Encoder Testing Program\n");
  Serial.print("Now setting up the Left Encoder: Pin ");
  Serial.print(EncoderMotorLeft);
  Serial.println();
  pinMode(EncoderMotorLeft, INPUT_PULLUP);  //set the pin to input
  // The following code sets up the PinChange Interrupt
  // Valid interrupt modes are: RISING, FALLING or CHANGE
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(EncoderMotorLeft),
                           indexLeftEncoderCount, CHANGE);
  // if you "really" want to know what's going on read the PinChange.h file :)
  /////////////////////////////////////////////////
  Serial.print("Now setting up the Right Encoder: Pin ");
  Serial.print(EncoderMotorRight);
  Serial.println();
  pinMode(EncoderMotorRight, INPUT_PULLUP);  //set the pin to input
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(EncoderMotorRight),
                           indexRightEncoderCount, CHANGE);
}  /////////////// end of setup ////////////////////////////////////
/////////////////////// loop() ////////////////////////////////////


void loop() {
  while (digitalRead(pushButton) == 1)
    ;  // wait for button push
  while (digitalRead(pushButton) == 0)
    ;  // wait for button release
  Serial.println("Button Pressed");
  for (int i = 0; i < sizeof(moves) / 2; i++) {  // Loop through entire moves list
    if (moves[i] == LEFT) {
      // Fill with code to turn left
      turn(LEFT, 90);
    } else if (moves[i] == RIGHT) {
      // Fill with code to turn right
      turn(RIGHT, 90);
    } else {
      // Fill with code to drive forward
      drive(moves[i]);
    }
    // TODO: use SW break from Lab 2
    run_motor(A, 0);
    run_motor(B, 0);
    delay(1000);
  }
}


//////////////////////////////// end of loop() /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int drive(float distance) {
  // create variables needed for this function
  int countsDesired, cmdLeft, cmdRight, xerr_L, xerr_R, dxerr_L, dxerr_R;
  float dt;
  // Find the number of encoder counts based on the distance given, and the
  // configuration of your encoders and wheels
  countsDesired = distance / DistancePerRev * EncoderCountsPerRev;
  // reset the current encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  // we make the errors greater than our tolerance so our first test gets us into the loop
  xerr_L = distTolerance + 1;
  xerr_R = distTolerance + 1;
  dxerr_L = 0;
  dxerr_R = 0;
  unsigned long t0 = millis();
  // Begin PID control until move is complete
  while (xerr_L > distTolerance || xerr_R > distTolerance) {
    dt = float(millis() - t0);
    t0 = millis();
    // Get PWM values from the controller function
    cmdLeft = PD_Controller(kP_A, kD_A, xerr_L, dxerr_L, dt, minPWM_A, maxPWM_A);
    cmdRight = PD_Controller(kP_B, kD_B, xerr_R, dxerr_L, dt, minPWM_B, maxPWM_B);
    // Run motors with calculated PWM values
    run_motor(A, cmdLeft);
    run_motor(B, cmdRight);
    // Update encoder error
    // Error is the number of encoder counts between here and the destination
    dxerr_L = (countsDesired - leftEncoderCount) - xerr_L;
    dxerr_R = (countsDesired - rightEncoderCount) - xerr_R;
    xerr_L = countsDesired - leftEncoderCount;
    xerr_R = countsDesired - rightEncoderCount;
    // Some print statements, for debugging
    Serial.print(xerr_L);
    Serial.print(" ");
    Serial.print(dxerr_L);
    Serial.print(" ");
    Serial.print(cmdLeft);
    Serial.print("\t");
    Serial.print(xerr_R);
    Serial.print(" ");
    Serial.print(dxerr_R);
    Serial.print(" ");
    Serial.println(cmdRight);
  }
}


////////////////////////////////////////////////////////////////////////////////
// Write a function for turning with PID control, similar to the drive function
int turn(int direction, float degrees) {
  // create variables needed for this function
  int countsDesired, cmdLeft, cmdRight, xerr_L, xerr_R, dxerr_L, dxerr_R;
  float dt;

  // Find the number of encoder counts based on the distance given, and the
  // configuration of your encoders and wheels
  countsDesired = degrees / DegreesPerRev * EncoderCountsPerRev;
  // reset the current encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  // we make the errors greater than our tolerance so our first test gets us into the
  loop
    xerr_L = distTolerance + 1;
  xerr_R = distTolerance + 1;
  dxerr_L = 0;
  dxerr_R = 0;
  unsigned long t0 = millis();
  // Begin PID control until move is complete
  while (xerr_L > distTolerance || xerr_R > distTolerance) {
    dt = float(millis() - t0);
    t0 = millis();
    // Get PWM values from controller function
    cmdLeft = PD_Controller(kP_A, kD_A, xerr_L, dxerr_L, dt, minPWM_A, maxPWM_A);
    cmdRight = PD_Controller(kP_B, kD_B, xerr_R, dxerr_L, dt, minPWM_B, maxPWM_B);
    // Run motors with calculated PWM values in the indicated direction
    run_motor(A, direction * cmdLeft);
    run_motor(B, -direction * cmdRight);
    // Update encoder error
    // Error is the number of encoder counts between here and the destination
    dxerr_L = (countsDesired - leftEncoderCount) - xerr_L;
    dxerr_R = (countsDesired - rightEncoderCount) - xerr_R;
    xerr_L = countsDesired - leftEncoderCount;
    xerr_R = countsDesired - rightEncoderCount;
    // Some print statements, for debugging
    Serial.print(xerr_L);
    Serial.print(" ");
    Serial.print(dxerr_L);
    Serial.print(" ");
    Serial.print(cmdLeft);
    Serial.print("\t");
    Serial.print(xerr_R);
    Serial.print(" ");
    Serial.print(dxerr_R);
    Serial.print(" ");
    Serial.println(cmdRight);
  }
}
//////////////////////////////////////////////////////////
// If you want, write a function to correct your path due
// to an active bump sensor. You will want to compensate
// for any wheel encoder counts that happend during this maneuver
//////////////////////////////////////////////////////////
int PD_Controller(float kP, float kD, int xerr, int dxerr, float dt, int minPWM, int maxPWM)
// gain, deadband, and error, both are integer values
{
  if (xerr <= distTolerance) {  // if error is acceptable, PWM = 0
    return (0);
  }


  // Proportional and Derivative control
  int pwm = int((kP * xerr) + (kD * (dxerr / dt)));
  pwm = constrain(pwm, minPWM, maxPWM);  // Bind value between motor's min and max
  return (pwm);
}


//////////////////////////////////////////////////////////
// These are the encoder interupt funcitons, they should NOT be edited
void indexLeftEncoderCount() {
  leftEncoderCount++;
}


//////////////////////////////////////////////////////////
void indexRightEncoderCount() {
  rightEncoderCount++;
}