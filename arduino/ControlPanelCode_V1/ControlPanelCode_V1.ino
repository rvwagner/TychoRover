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
#define ROS

#ifdef ROS
#include <ros.h>
#include <tycho/DisplayPanel.h>
ros::NodeHandle nh;
#endif

// Assign each display a pin on the arduino.
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
  // --------Initialize SPI pins
  pinMode(speedPin, OUTPUT);
  pinMode(pitchPin, OUTPUT);
  pinMode(headingPin, OUTPUT);
  pinMode(tempPin, OUTPUT);
  digitalWrite(speedPin, HIGH);
  digitalWrite(pitchPin, HIGH);
  digitalWrite(headingPin, HIGH);
  digitalWrite(tempPin, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  
  clearDisplaySPI();
  setDecimalsSPI(speedPin, 0b111111); 
  
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
  demoLoop();
  
  //nh.spinOnce();
  delay(10);
}

///////////////////////
// Display Functions //
///////////////////////
void displaySpeed(float tykeSpeed)
{
   int floatInInt;
   
   if(tykeSpeed <=1)
   {
      floatInInt = (int)(tykeSpeed * 1000); 
   }
   
   floatInInt = (int)(tykeSpeed * 100);
   
   sprintf(tempString, "%4d", floatInInt);
   segStringSPI(speedPin, tempString);
   
   if(tykeSpeed < 100)
      setDecimalsSPI(speedPin, 0b00000010); 
   else
     setDecimalsSPI(speedPin, 0b00000100);
}

void displayHeading(float tykeHeading)
{
  int floatInInt;
  if(tykeHeading <= 1)
    floatInInt = (int)(tykeHeading * 1000);
  else
    floatInInt = (int)(tykeHeading * 100);
  
  sprintf(tempString, "%4d", floatInInt);
  segStringSPI(headingPin, tempString);
  
  if(tykeHeading < 100)
    setDecimalsSPI(headingPin, 0b00000010);
  else
    setDecimalsSPI(headingPin, 0b00000100);
    
}

void displayRoll(float roll)
{
  int floatInInt;
  
  floatInInt = (int)(roll * 100);
  
  sprintf(tempString, "%4d", floatInInt);
  segStringSPI(pitchPin, tempString);
  
  if(roll < 100)
    setDecimalsSPI(pitchPin, 0b00000010);
  else
    setDecimalsSPI(pitchPin, 0b00000100);
    
}

void displayPitch(float pitch)
{
  int floatInInt;
  
  floatInInt = (int)(pitch * 100);
  
  sprintf(tempString, "%4d", floatInInt);
  segStringSPI(pitchPin, tempString);
  
  if(pitch < 100)
    setDecimalsSPI(pitchPin, 0b00000010);
  else
    setDecimalsSPI(pitchPin, 0b00000100);
}

void displayTemp(float tykeTemp)
{
  int floatInInt;
  
  if(tykeTemp <= 1)
    floatInInt = (int)(tykeTemp * 1000);
  else
    floatInInt = (int)(tykeTemp * 100);
  
  sprintf(tempString, "%4d", floatInInt);
  segStringSPI(tempPin, tempString);
  
  if(tykeTemp < 100)
    setDecimalsSPI(tempPin, 0b00000010);
  else
    setDecimalsSPI(tempPin, 0b00000100);
    
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
  digitalWrite(headingPin, LOW);
  digitalWrite(tempPin, LOW);
  SPI.transfer(0x76); // Clear display command
  digitalWrite(speedPin, HIGH);
  digitalWrite(pitchPin, HIGH);
  digitalWrite(headingPin, HIGH);
  digitalWrite(tempPin, HIGH);
}

// Function to set the brightness of displays
void setBrightnessSPI(byte value)
{
  digitalWrite(speedPin, LOW);
  digitalWrite(pitchPin, LOW);
  digitalWrite(headingPin, LOW);
  digitalWrite(tempPin, LOW); 
  SPI.transfer(0x7A);  // Set brightness command byte
  SPI.transfer(value); // Brightness data byte
  digitalWrite(speedPin, HIGH);
  digitalWrite(pitchPin, HIGH);
  digitalWrite(headingPin, HIGH);
  digitalWrite(tempPin, HIGH);
}

// Function to set the decimals for the displays
void setDecimalsSPI(const int pin, byte decimals)
{
  digitalWrite(pin, LOW);
  SPI.transfer(0x77); // Set decimal command byte
  SPI.transfer(decimals);
  digitalWrite(pin, HIGH);

}

/////////////////////
// Dummy Data Loop //
/////////////////////
void demoLoop()
{
  displaySpeed(50.36);
  displayRoll(10.21);
  displayTemp(120.1);
  displayHeading(66.6);
  delay(2000);
  clearDisplaySPI();
  displaySpeed(123.34);
  displayRoll(100);
  displayTemp(12.54);
  displayHeading(77.7);
  delay(2000);
}




