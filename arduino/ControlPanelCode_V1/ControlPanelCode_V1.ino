
/*
  Circuit:            7 Segment Display 
  5v   -------------------- VCC
  GND  -------------------- GND
  32   -------------------- SS (speed display)
  34   -------------------- SS (pitch display)
  35   -------------------- SS (roll display)
  36   -------------------- SS (heading display)
  37   -------------------- SS (temperature display)
  52   -------------------- SCK
  51   -------------------- SDI
  
  Circuit:             LED Matrix Display
  30   -------------------- DIN 
  33   -------------------- CLK
  31   -------------------- CS
    
*/

#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include "LedControl.h"

//Create LSM9DS1 object
LSM9DS1 imu;

#define ROS

#define LSM9DS1_M    0x1E
#define LSM9DS1_AG   0x6B

#define PRINT_CALCULATED
#define DECLINATION -10.13 //Tempe Arizona Declination
#define PRINT_SPEED 2000

#ifdef ROS
#include <ros.h>
#include <tycho/DisplayPanel.h>
ros::NodeHandle nh;
#endif

//Pins: DIN,CLK,CS, # of displays connected
LedControl lc = LedControl(30,33,31,1);

// Assign each warning motor a pin.
const int temp_warning = 3; //Motor 1
const int slope_warning = 5; //Motor 2
const int speed_warning = 7; //Motor 3
const int batt_warning = 9; // Motor 4
const int brakes_engaged = 11; // Motor 5

// Assign each display a pin.
const int speedPin = 32;
const int pitchPin = 34;
const int rollPin = 35;
const int headingPin = 36;
const int tempPin = 37;

float wheelSpeed = 0;
float front_left_angle = 0;
float front_right_angle = 0;
float back_left_angle = 0;
float back_right_angle = 0;
float total_amps = 0;
float battery_pct = 0;
float max_drive_temp = 0;
float roll = 0;
float pitch = 0;
float heading = 0;
//boolean baking = false;

int hottest_motor_id = 0;
char tempString[10]; // char array for printing data to displays

Servo servos[4];

///////////////////
// ROS callbacks //
///////////////////
#ifdef ROS
void rosSetDisplayValues(const tycho::DisplayPanel& values){
  
  displayValue(speedPin, values.speed);
  displayValue(pitchPin, retPitch(imu.ax, imu.ay, imu.az));
  displayValue(rollPin, retRoll(imu.ay, imu.az));
  displayValue(headingPin, retHeading(-imu.my, -imu.mx));
  displayValue(tempPin, values.max_drive_temp);
  
    /* Handle warning flags on arduino?
  if(values.speed > xx)
  {
    servos[0].write(90);
  }else{
    servos[0].write(0); 
  }
  if(values.pitch > xx || values.roll > xx)
  {
    servos[1].write(90);  
  }else{
    servos[1].write(0); 
  }
  if(values.
  
  if(values.max_drive_temp > xx)
  {
    servos[2].write(90);
  }else{
    servos[2].write(0); 
  }
  
  if(values.batt_temp < xx)
  {
    servos[3].write(90); 
  }else{
    servos[3].write(0);
  }
  
  if(values.brakes_engaged = 
  */
  
  display_angles(values.front_left_angle, values.front_right_angle, values.back_left_angle, values.back_right_angle);
  
}

ros::Subscriber<tycho::DisplayPanel> displaySub("tycho/display_values", &rosSetDisplayValues);

#endif

////////////////////
// Setup and Loop //
////////////////////
void setup()
{
  
  //Configure LSM9DS1
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  
  if(!imu.begin()) 
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while(1)
      ;
  }
  
  servos[0].attach(temp_warning);
  servos[1].attach(slope_warning);
  servos[2].attach(speed_warning);
  servos[3].attach(batt_warning);
  servos[4].attach(brakes_engaged);
  
  lc.shutdown(0, false);
  lc.setIntensity(0,5);
  lc.clearDisplay(0);
 
  // --------Initialize SPI pins
  pinMode(speedPin, OUTPUT);
  pinMode(pitchPin, OUTPUT);
  pinMode(rollPin, OUTPUT);
  pinMode(headingPin, OUTPUT);
  pinMode(tempPin, OUTPUT);
  digitalWrite(speedPin, HIGH);
  digitalWrite(pitchPin, HIGH);
  digitalWrite(rollPin, HIGH);
  digitalWrite(headingPin, HIGH);
  digitalWrite(tempPin, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  
  setBrightnessSPI(255);
 
  #ifndef ROS
    Serial.begin(9600);
  #else 
    nh.initNode();
    nh.subscribe(displaySub);
    Serial.begin(9600);
  #endif
  
}

void loop()
{
  updateAccVals();
  displayValue(pitchPin, retPitch(imu.ax, imu.ay, imu.az));
  displayValue(rollPin, retRoll(imu.ay, imu.az));
  displayValue(headingPin, retHeading(-imu.my, -imu.mx));
  //demoLoop();
  nh.spinOnce();
  delay(10);

}
/////////////////////
// Dummy Data Loop //
/////////////////////
void demoLoop()
{
  
  displayValue(speedPin, .36);
  displayValue(tempPin, 120.2);
  
  delay(200);
}

void updateAccVals()
{
  imu.readMag();
  imu.readAccel(); 
  delay(200);
}

float retRoll(float ay, float az)
{
  roll = atan2(ay, az);
  roll *= 180.0/PI;
 
  return roll; 
}

float retPitch(float ax, float ay, float az)
{
  pitch = atan2(-ax, sqrt(ay * ay + az * az));
  pitch *= 180.0/PI;
  
  return pitch;
}

float retHeading(float my, float mx)
{
  
  if(my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI/180;
  
  if(heading > PI) heading -= (2*PI);
  
  else if (heading < -PI) heading += (2 * PI);
  
  else if (heading < 0) heading += 2 * PI;
  
  //Convert from radians to degrees
  heading *= 180.0 / PI;
  
  Serial.print("Heading: " ); 
  Serial.println(heading);
  
  return heading;
} 
///////////////////////
// Display Functions //
///////////////////////

void displayValue(int pinNumber, float dispValue)
{
    clearDisplaySPI(pinNumber);
    
    int floatInInt;
    
    setDecimalsSPI();
    
    floatInInt = (int)(dispValue * 10); 

    sprintf(tempString, "%4d", floatInInt);
    segStringSPI(pinNumber, tempString);
    
    setDecimalsSPI();
  
}

////////////////////
// Display Angles //
////////////////////
byte degree0[] =
{
  B00000010,
  B00000010,
  B00000010
};

byte degree45[] =
{
  B00000001,
  B00000010,
  B00000100
};

byte degree_neg45[] =
{
  B00000100,
  B00000010,
  B00000001
};

byte degree90[] =
{
  B00000000,
  B00000111,
  B00000000
};

void display_angles(float front_left_angle, float front_right_angle, float back_left_angle, float back_right_angle) {
 
  delay(100);
 byte fullDisplay[] =
{
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000
};

  //top right
  for(int i = 0; i < 3; i++)
  {
    fullDisplay[i] = fullDisplay[i] | wheelAngle(front_right_angle)[i];
  } 
  
  //bottom right
  for(int i = 0; i < 3; i++)
  {
    fullDisplay[i+5] = fullDisplay[i+5] | wheelAngle(back_right_angle)[i]; 
  }
  
  //top left
  for(int i = 0; i < 3; i++)
  {
    fullDisplay[i] = fullDisplay[i] | wheelAngle(front_left_angle)[i] << 5;
  }
  
  //bottom left
  for (int i = 0; i < 3; i++)
  {
    fullDisplay[i+5] = fullDisplay[i+5] | wheelAngle(back_left_angle)[i] << 5; 
  } 
  
  for(int i = 0; i < 8; i++){
    lc.setRow(0, i, fullDisplay[i]); 
  }
}
//////////////////////
// Helper Functions //
//////////////////////

// Function to print array of chars
void segStringSPI(const int pin, String toSend)
{
  digitalWrite(pin, LOW);
  
  for(int i = 0; i < 4; i++)
  {
     SPI.transfer(toSend[i]); 
  }
  digitalWrite(pin, HIGH);
}

// Function to clear all displays
void clearDisplaySPI(int pin)
{
  
  digitalWrite(pin, LOW);
  SPI.transfer(0x76);
  digitalWrite(pin, HIGH);
  
}

// Function to set the brightness of displays
void setBrightnessSPI(byte value)
{
  digitalWrite(speedPin, LOW);
  digitalWrite(pitchPin, LOW);
  digitalWrite(rollPin, LOW);
  digitalWrite(headingPin, LOW);
  digitalWrite(tempPin, LOW); 
  SPI.transfer(0x7A);  // Set brightness command byte
  SPI.transfer(value); // Brightness data byte
  digitalWrite(speedPin, HIGH);
  digitalWrite(pitchPin, HIGH);
  digitalWrite(rollPin, HIGH);
  digitalWrite(headingPin, HIGH);
  digitalWrite(tempPin, HIGH);
}

// Function to set the decimals for the displays
void setDecimalsSPI()
{
  digitalWrite(speedPin, LOW);
  digitalWrite(pitchPin, LOW);
  digitalWrite(rollPin, LOW);
  digitalWrite(headingPin, LOW);
  digitalWrite(tempPin, LOW);
  SPI.transfer(0x77); // Set decimal command byte
  SPI.transfer(0b00000100);// Set to 1 decimal place
  digitalWrite(speedPin, HIGH);
  digitalWrite(pitchPin, HIGH);
  digitalWrite(rollPin, HIGH);
  digitalWrite(headingPin, HIGH);
  digitalWrite(tempPin, HIGH);
}

// Function to return wheel angles for the matrix display. 
byte* wheelAngle(float angle)
{
  
  if((angle >= 0 && angle < 30) || (angle < 0 && angle > -30))  
  {
    return degree0; 
  }
  else if(angle >= 30 && angle < 75)
  {
    return degree45; 
  }
  else if(angle <= -30 && angle > -75) 
  {
    return degree_neg45;
  }
  else if(angle > -90 && angle <= -135)
  {
    return degree_neg45;
  }
  else if(angle >= 75 && angle <= 90 || (angle <= -75 && angle >= -90))
  {
    return degree90;
  } 
}









