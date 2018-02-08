
/*
  Circuit:            7 Segment Display 
  5v   -------------------- VCC
  GND  -------------------- GND
  31   -------------------- SS (speed display)
  33   -------------------- SS (slope display)
  35   -------------------- SS (heading display)
  37   -------------------- SS (temperature display)
  52   -------------------- SCK
  51   -------------------- SDI
    
*/

#include <SPI.h>
#include "LedControl.h"

#define ROS

#ifdef ROS
#include <ros.h>
#include <tycho/DisplayPanel.h>
ros::NodeHandle nh;
#endif

// Assign each display a pin on the arduino.
LedControl lc = LedControl(30,33,31,1);

unsigned long delayTime = 200; //For testing purposes

const int speedPin = 31;
const int pitchPin = 33;
const int rollPin = 34;
const int headingPin = 35;
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

int hottest_motor_id = 0;

char tempString[10]; // char array for printing data to displays

///////////////////
// ROS callbacks //
///////////////////
#ifdef ROS
void rosSetDisplayValues(const tycho::DisplayPanel& values)
{

}

ros::Subscriber<tycho::DisplayPanel> displaySub("tycho/display_values", &rosSetDisplayValues);

#endif

////////////////////
// Setup and Loop //
////////////////////
void setup()
{
  lc.shutdown(0, false);
  lc.setIntensity(0,5);
  lc.clearDisplay(0);
 
  // --------Initialize SPI pins
  pinMode(speedPin, OUTPUT);
  pinMode(pitchPin, OUTPUT);
  //pinMode(rollPin, OUTPUT);
  pinMode(headingPin, OUTPUT);
  pinMode(tempPin, OUTPUT);
  digitalWrite(speedPin, HIGH);
  digitalWrite(pitchPin, HIGH);
  //digitalWrite(rollPin, HIGH);
  digitalWrite(headingPin, HIGH);
  digitalWrite(tempPin, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  
  clearDisplaySPI();
  
  setBrightnessSPI(255);
  
  clearDisplaySPI();
    
  #ifndef ROS
    Serial.begin(57600);
  #else 
    nh.initNode();
    nh.subscribe(displaySub);
  #endif
  
}

void loop()
{
  display_angles(45, 130, 0, 45);
  
  //nh.spinOnce();
  delay(10);
}

///////////////////////
// Display Functions //
///////////////////////

void displayValue(int pinNumber, float dispValue)
{
    int floatInInt;
    
    setDecimalsSPI();
    
    if(dispValue <= 1) 
    {
       floatInInt = (int)(dispValue * 100); 
    } 
    else {
      floatInInt = (int)(dispValue * 10);
    }
    
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

byte degree135[] =
{
  B00000001,
  B00000010,
  B00000100
};

void display_angles(float front_left_angle, float front_right_angle, float back_left_angle, float back_right_angle) {
  
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
void clearDisplaySPI()
{
  digitalWrite(speedPin, LOW);
  digitalWrite(pitchPin, LOW);
  digitalWrite(rollPin, LOW);
  digitalWrite(headingPin, LOW);
  digitalWrite(tempPin, LOW);
  SPI.transfer(0x76); // Clear display command
  digitalWrite(speedPin, HIGH);
  digitalWrite(pitchPin, HIGH);
  digitalWrite(rollPin, HIGH);
  digitalWrite(headingPin, HIGH);
  digitalWrite(tempPin, HIGH);
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
  if(angle >= 0 && angle < 45)
  {
    return degree0; 
  }
  else if(angle >= 45 && angle < 90)
  {
    return degree45; 
  }
  else if(angle >= 90 && angle < 120)
  {
    return degree90;
  } 
  else if(angle >= 120 && angle < 135)
  {
    return degree135;
  }
}

/////////////////////
// Dummy Data Loop //
/////////////////////
void demoLoop()
{
  
  displayValue(speedPin, 50.36);
  displayValue(pitchPin, 1.52);
  displayValue(tempPin, 120.2);
  displayValue(headingPin, 84.3);
  
  delay(3000);
  //displaySpeed(50.36);
  //displayRoll(10.21);
  //displayTemp(120.1);
  //displayHeading(66.6);
}




