/* 


Function usage:
For direct dumb control over the motors, use the following functions for driving and steering:
setAllMotorSpeeds(int spd1, int spd2, int spd3, int spd4, int turnInPlace)
setSteeringAngles(int a1, int a2, int a3, int a4)
These simply set the motor speeds and steering angles as you command, without trying to ensure
that the rover state remains internally consistent or physically possible.  The associated ROS
topics are "motorValues" and "motorAngles", both of which are 4-value Int16MultiArrays.


For higher-level control, command a speed with 
setVehicleSpeed(int spd)
and define steering angles with either of the following, depending on whether you want to turn
or strafe/drive straight:
setAnglesFromTurnCenter(float turnCenterX, float turnCenterY)
float setAnglesStrafing(float angle)
These functions act to ensure that the wheels on the rover are at appropriate and useful angles,
and that the wheel speeds are scaled to minimize slippage.  The associated ROS topics are 
"vehicleDirection": Int16MultiArray, three values: strafingAngle (degrees; ignored if <-90),
                    turnCenterX, turnCenterY (both in cm; ignored if strafingAngle >= -90)
"vehicleSpeed": Int16, range -100 to +100.  If spinning in place, positive is spinning CCW.

*/
#define ROS

#ifdef ROS
#include <ros.h>
#include <tycho/WheelAnglesSpeeds.h>
ros::NodeHandle  nh;
#endif

#include <Servo.h>

const int leftMotorsForwardPin   = 2; // Motors 1+3 (right side), 2+4 (left side) will always
const int leftMotorsBackwardPin  = 4; // spin in the same direction, although not at the same
const int rightMotorsForwardPin  = 7; // speed, saving four pins
const int rightMotorsBackwardPin = 8;

const int frontRightMotorSpeedPin = 3;  // Motor 1
const int frontLeftMotorSpeedPin  = 5;  // Motor 2
const int backRightMotorSpeedPin  = 6;  // Motor 3
const int backLeftMotorSpeedPin   = 11; // Motor 4 // Servo library disables PWM on pins 9 and 10

const int frontRightServoPin = 9; // Those saved pins are used for steering servos
const int frontLeftServoPin  = 10;
const int backRightServoPin  = 12;
const int backLeftServoPin   = 13;

const int deadband = 0; // Ignore speed commands between -deadband and +deadband

Servo s1;
Servo s2;
Servo s3;
Servo s4;
Servo servos[4];
int servo_offsets[4] = {-8, -5, -5, 6}; // Add to 90 to get the actual zero-point

float jointX[4] = {11.7, 11.7, -11.7, -11.7}; // +X is forward, numbers in cm
float jointY[4] = {-6.35, 6.35, -6.35, 6.35}; // +Y is left
float steeringArmLength = 3.65; // Approximate distance from turning point to the 

float speedMultipliers[4] = {1.0, 1.0, 1.0, 1.0};

int currentSpeed = 0; // Stores the last commanded speed value

int posX = 0;
int posY = 0;
int spd  = 0;
int straight = 1;


struct Motor {
  int fwdPin; // Any digital out -> IN1/IN3
  int bckPin; // Any digital out -> IN2/IN4
  int spdPin; // Any PWM -> ENA/ENB
  char curSpd; // Range -100 to +100
} m1, m2, m3, m4;

void initMotor(struct Motor *m, int s, int f, int b){
  m->fwdPin = f;
  m->bckPin = b;
  pinMode(m->fwdPin, OUTPUT);
  pinMode(m->bckPin, OUTPUT);
  if(s>=-1){
    m->spdPin = s;
    pinMode(m->spdPin, OUTPUT);
  }
  m->curSpd = 0;
}


/////////////
// DRIVING //
/////////////

// Set motor speed on a scale of -100 to +100
void setMotorSpeed(int spd, struct Motor m){
  m.curSpd = spd;
  if(m.fwdPin >= 0 && m.bckPin >= 0){
    if(spd > deadband){
      digitalWrite(m.fwdPin, HIGH);
      digitalWrite(m.bckPin, LOW); 
    } else if (spd < deadband) {
      digitalWrite(m.fwdPin, LOW);
      digitalWrite(m.bckPin, HIGH);
    } else { // Center deadband
      digitalWrite(m.fwdPin, LOW);
      digitalWrite(m.bckPin, LOW);
    }
  }
  //Serial.print( (50+abs(spd)*2), DEC); Serial.print(" "); Serial.println(spd, DEC);
  
  // Optionally set a PWM signal for the motor speed
  // Limits output to 255, just in case the input spd is >100
  if(m.spdPin >= 0) analogWrite(m.spdPin, min((50+abs(spd)*2), 255) );
}

// Directly set the speeds of all motors
// Speed arguments can be in the range -100 to +100
// If turnInPlace is true, the left-hand wheel directions will be automatically reversed.
void setAllMotorSpeeds(int spd1, int spd2, int spd3, int spd4, int turnInPlace){
  int leftMultiplier=1;
  if(turnInPlace) leftMultiplier = -1; // Reverse the left-hand motors to turn in place
  
  // Approximation of the current center-of-vehicle speed, needed for higher-level command system
  // Ideally these systems won't be used together, but just in case...
  // TODO: Calculate actual center-of-vehicle speed based on wheel angles?
  currentSpeed = (spd1+spd2+spd3+spd4)/4;
  
  setMotorSpeed(spd1               , m1);
  setMotorSpeed(spd2*leftMultiplier, m2);
  setMotorSpeed(spd3               , m3);
  setMotorSpeed(spd4*leftMultiplier, m4);
}

// Set the motor speeds such that the center of the vehicle travels at the commanded speed.
// Valid speed values range from -100 to +100
// Or, if the vehicle is turning about a point within the frame, sets the speed of
// the outer-most wheel to the commanded speed (and other speeds are scaled accordingly).
void setVehicleSpeed(int spd){
  currentSpeed = spd;
  // Set individual wheel speeds based on multiplier array
  // That array inherently takes care of the direction-reversal for spin-in-place
  int modifiedSpeeds[4] = {(int)(spd*speedMultipliers[0]),
                           (int)(spd*speedMultipliers[1]),
                           (int)(spd*speedMultipliers[2]),
                           (int)(spd*speedMultipliers[3])};
  
  // If any wheel is above max speed, scale all speeds down
  int maxSpd = 0;
  for (int i = 0; i < 4; i++) if(abs(modifiedSpeeds[i]) > maxSpd) maxSpd = abs(modifiedSpeeds[i]);
  if (maxSpd > 100) {
    float scaleFactor = 100.0 / maxSpd;
    Serial.println(scaleFactor, DEC);
    for (int i = 0; i < 4; i++) modifiedSpeeds[i] *= scaleFactor;
  }

  /* Serial.print("Speed ");
  Serial.print(modifiedSpeeds[0], DEC);  Serial.print(" ");
  Serial.print(modifiedSpeeds[1], DEC);  Serial.print(" ");
  Serial.print(modifiedSpeeds[2], DEC);  Serial.print(" ");
  Serial.print(modifiedSpeeds[3], DEC);  Serial.println(); */
  
  // Command speed controller
  setMotorSpeed(modifiedSpeeds[0], m1);
  setMotorSpeed(modifiedSpeeds[1], m2);
  setMotorSpeed(modifiedSpeeds[2], m3);
  setMotorSpeed(modifiedSpeeds[3], m4);
}



//////////////
// STEERING //
//////////////



// Commands the servos to go to specified angles
// Returns the estimated time until they reach that angle in ms
int setSteeringAngles(int a1, int a2, int a3, int a4){
  // Get amouont of maximum change in angle for calculating update duration
  int maxAngleChange = abs(a1 - servos[0].read());
  int change = abs(a2 - servos[1].read());
  if (change > maxAngleChange) maxAngleChange = change;
  change = abs(a3 - servos[2].read());
  if (change > maxAngleChange) maxAngleChange = change;
  change = abs(a4 - servos[3].read());
  if (change > maxAngleChange) maxAngleChange = change;
  
  servos[0].write(a1+servo_offsets[0]);
  servos[1].write(a2+servo_offsets[1]);
  servos[2].write(a3+servo_offsets[2]);
  servos[3].write(a4+servo_offsets[3]);

  return maxAngleChange*2; // Servo nominally rotates at 500deg/sec
}

// Commands the steering servos to go to appropriate angles for the vehicle to turn
// about the specified position in x/y space (+X is forward, +Y is left, units in cm)
// Also adjusts the current motor speeds to be correct for the differing ditances from
// the turning center (WARNING: Does not play nicely with setAllMotorSpeeds; use 
// setVehicleSpeed instead).
// Returns estimated time until the wheels reach the target angles in ms.
int setAnglesFromTurnCenter(float turnCenterX, float turnCenterY){
  int angles[4] = {0,0,0,0};
  float turnPointDistance = sqrt(turnCenterX*turnCenterX + turnCenterY*turnCenterY);
  float maxMultiplier = 0.0;
  for (int i = 0; i < 4; i++){
    float distY = turnCenterY-jointY[i];
    float distX = turnCenterX-jointX[i];
    angles[i] = atan(distX/distY)*180/3.14+90;
    
    // Calculate the distance to the turning point based on whether steering arm points 
    // towards or away from the turn point
    float distTotal = sqrt(distX*distX + distY*distY);
    if (turnCenterY/jointY[i] > 1){ // Same sign and abs(turnCenterY) > abs(self.jointY)
      distTotal -= steeringArmLength;
    } else { // Arm points away from point
      distTotal += steeringArmLength;
    }

    // For very small offsets from turn-in-place, make all wheel speeds match here to avoid rounding issues
    if (turnPointDistance <= 0.01)  speedMultipliers[i] = 1.0;
    else speedMultipliers[i] = distTotal/turnPointDistance;
    if (speedMultipliers[i] > maxMultiplier) maxMultiplier = speedMultipliers[i];
  }
  
  // If turn point is interior to wheels, adjust multipliers to:
  // A) Max out at 1.0
  // B) Reverse the left-hand wheels (2 and 4; indices 1 and 3)
  // TODO: Add variables for joint X and Y positioins in general, rather than relying on joint[1] being positive
//  Serial.print(abs(turnCenterX), DEC); Serial.print(" ");
//  Serial.print(jointX[1], DEC); Serial.print(" ");
//  Serial.print(abs(turnCenterY), DEC); Serial.print(" ");
//  Serial.print(jointY[1], DEC); Serial.println();
  if ( abs(turnCenterX) < jointX[1] && abs(turnCenterY) < jointY[1] ) {
    speedMultipliers[0] =  speedMultipliers[0] / maxMultiplier;
    speedMultipliers[1] = -speedMultipliers[1] / maxMultiplier;
    speedMultipliers[2] =  speedMultipliers[2] / maxMultiplier;
    speedMultipliers[3] = -speedMultipliers[3] / maxMultiplier;
  }

  
  
  //Serial.print("Mults ");
  //Serial.print(speedMultipliers[0], DEC);  Serial.print(" ");
  //Serial.print(speedMultipliers[1], DEC);  Serial.print(" ");
  //Serial.print(speedMultipliers[2], DEC);  Serial.print(" ");
  //Serial.print(speedMultipliers[3], DEC);  Serial.println();
  
  //Serial.print("Angle ");
  //Serial.print(angles[0], DEC);  Serial.print(" ");
  //Serial.print(angles[1], DEC);  Serial.print(" ");
  //Serial.print(angles[2], DEC);  Serial.print(" ");
  //Serial.print(angles[3], DEC);  Serial.println(); 

  // Adjust the current drive speed based on the new speed multipliers
  setVehicleSpeed(currentSpeed);

  // Actually command the servos
  return setSteeringAngles(angles[0], angles[1], angles[2], angles[3]);
}

// Set the steering angle to put all wheels parallel at the specified angle
// Valid angles go from -90 to +90, with 0 being straight ahead and +90 being to the left
// Returns estimated time until the wheels reach the target angles in ms.
float setAnglesStrafing(float angle){
  // No speed multiplication needed in this drive mode
  speedMultipliers[0] = 1.0;
  speedMultipliers[1] = 1.0;
  speedMultipliers[2] = 1.0;
  speedMultipliers[3] = 1.0;
  
  // Adjust the current drive speed based on the new speed multipliers
  setVehicleSpeed(currentSpeed);
  
  return setSteeringAngles(angle+90, angle+90, angle+90, angle+90);
}


/////////////////////////
// Autopilot Demo Loop //
/////////////////////////

void demoLoop(){
  
  // Demo loop
  setVehicleSpeed(0); delay(200);

  // Drive forward
  delay(setAnglesStrafing(0)); // Servos take ~1/4s to rotate into position
  setVehicleSpeed(50);
  delay(1000);
  
  // pause
  setVehicleSpeed(0); delay(200);

  // Spin in place 1/4 turn
  delay(setAnglesFromTurnCenter(0,0));
  setVehicleSpeed(-50);
  delay(500);

  // Pause
  setVehicleSpeed(0); delay(200);

  // Drive in a half-circle
  delay(setAnglesFromTurnCenter(0, -23));
  setVehicleSpeed(50);
  delay(3000);

  // Pause
  setVehicleSpeed(0); delay(200);

  // TUrn back 3/4 turn
  delay(setAnglesFromTurnCenter(0,0));
  setVehicleSpeed(50);
  delay(1500);
}


///////////////////
// ROS callbacks //
///////////////////

#ifdef ROS
void rosSetMotorSpeedAngles(const tycho::WheelAnglesSpeeds& values) {
  setMotorSpeed(values.front_right_speed*100, m1);
  setMotorSpeed(values.front_left_speed*100, m2);
  setMotorSpeed(values.back_right_speed*100, m3);
  setMotorSpeed(values.back_left_speed*100, m4);
//}
//void rosSetMotorAngles(const tycho::WheelAnglesSpeeds& values) {
  setSteeringAngles(values.front_right_angle+90, 
                    values.front_left_angle+90, 
                    values.back_right_angle+90, 
                    values.back_left_angle+90);
}

// Values are strafingAngle (ignored if <-90), turnCenterX, TurnCenterY (both ignored if strafingAngle >= -90)
//void rosSetVehicleDirection(const tycho::WheelAnglesSpeeds& values) {
//  if(values.data[0] >= -90) {
//    setAnglesStrafing(values.data[0]);
//  } else {
//    setAnglesFromTurnCenter(values.data[1], values.data[2]);
//  }
//  // TODO: Publish actual angles and speeds
//}
//void rosSetVehicleSpeed(const std_msgs::Int16& value) {
//  setVehicleSpeed(value.data);
//}

ros::Subscriber<tycho::WheelAnglesSpeeds> motorSub("tycho/low_level_motor_values",&rosSetMotorSpeedAngles);
//ros::Subscriber<std_msgs::Int16MultiArray> angleSub("motorAngles",&rosSetMotorAngles);
//ros::Subscriber<std_msgs::Int16MultiArray> directionSub("vehicleDirection",&rosSetVehicleDirection);
//ros::Subscriber<std_msgs::Int16MultiArray> speedSub("vehicleSpeed",&rosSetVehicleSpeed);
#endif







////////////////
// Setup/Loop //
////////////////

void setup() {
  initMotor(&m1, frontRightMotorSpeedPin, rightMotorsForwardPin, rightMotorsBackwardPin); // SpeedPin, ForwardPin, BackPin
  initMotor(&m2, frontLeftMotorSpeedPin,  leftMotorsForwardPin,  leftMotorsBackwardPin);
  initMotor(&m3, backRightMotorSpeedPin,  rightMotorsForwardPin, rightMotorsBackwardPin);
  initMotor(&m4, backLeftMotorSpeedPin,   leftMotorsForwardPin,  leftMotorsBackwardPin);
  servos[0].attach(frontRightServoPin);
  servos[1].attach(frontLeftServoPin);
  servos[2].attach(backRightServoPin);
  servos[3].attach(backLeftServoPin);
  setSteeringAngles(90, 90, 90, 90);
  
#ifndef ROS
  Serial.begin(57600);
#else
  nh.initNode();
  nh.subscribe(motorSub);
//  nh.subscribe(angleSub);
//  nh.subscribe(directionSub);
//  nh.subscribe(speedSub);
#endif
}

void loop() {
  
//  demoLoop();
//  return;
  
#ifndef ROS
  // If not doing ROS version, check for commands on the serial port
  // WASD moves steering point, Q/E set forward-backward, Space stops.
  while (Serial.available() > 0){
    char c = Serial.read();
    switch(c) {
      case 'w': posX++; break;
      case 'a': posY+=3; break;
      case 's': posX--; break;
      case 'd': posY-=3; break;
      case 'c': straight = !straight; break;
      case 'q': spd++; break;
      case 'e': spd--; break;
      case ' ': spd=   0; break;
    }
    Serial.print(spd, DEC);
    Serial.print(" ");
    Serial.print(posX, DEC);
    Serial.print(" ");
    Serial.println(posY, DEC);

    if (straight) setAnglesStrafing(-posY);
    else setAnglesFromTurnCenter(posX,posY);
    
    setVehicleSpeed(spd);
  }
#else
  // If doing ROS version, just wait for more inputs on the command topics
  nh.spinOnce();
  delay(10);
#endif
}

