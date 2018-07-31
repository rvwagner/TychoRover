/* 
 *  Tycho control panel interface code
 *  
 *  Handles interactions with the joystick and buttons on the control panel, and
 *  publishes three ROS topics:
 *  
 * tycho/joy (sensor_msgs/Joy):
 *    Standard joystick topic for the joystick and drive mode buttons
 *  tycho/gui_page (std_msgs/Int16):
 *    Integer indicating which page of the GUI to display
 *  tycho/mouse (???):
 *    As-yet-undefined topic for controlling an onscreen mouse using the joystick
 *    
 *  Organization:
 *  classes.h 
 *    contains all the code for interfacing with and calibrating the physical buttons
 *  drive.ino
 *    contains the logic for handling the interface while driving
 *  mouse.ino
 *    contains the logic for handling the interface while using the joystick as a mouse 
 *  control_panel_joystick.ino
 *    contains the general logic common to both states, along with setup
 *    
 */

#include "classes.h"

////////////////
// SET UP ROS //
////////////////

#define ROS
#include <ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>
#include <tycho/Mouse.h>
ros::NodeHandle nh;

std_msgs::Int16 page_msg;
ros::Publisher pagePub("tycho/gui_page", &page_msg);

// joy message type has variable-length arrays that must be allocated and assigned separately
// http://wiki.ros.org/rosserial/Overview/Messages
sensor_msgs::Joy joy_msg;
ros::Publisher joyPub("tycho/joy", &joy_msg);

float axes[]={0,0};
int32_t buttons[]={0,0,0,0,0,0};


tycho::Mouse mouse_msg;
ros::Publisher mousePub("tycho/mouse", &mouse_msg);


const int minSendInterval = 1000 / 50; // second number is min update rate in Hz




// Assorted buttons
/* Modes:
ID  Pin Mode
 5   x  Normal
 1   x  Strafe
 3   x  Turn-in-Place
 4   x  Circle-Strafe
 0   x  Unassigned
 2   x  Stop

 There is no longer any "reverse active" mode.  It was just confusing.
 We will see if I regret this...
 On the whiteboard that button is also called "reset position", which suggests a bug in joy2command
*/
int drive_mode_count = 6;
int drive_mode_pins[]             = {26,27,28,29,30,31};
int drive_mode_light_pins[]       = { 6, 7, 8, 9,10,11};
const int drive_mode_button_ids[] = { 5, 1, 3, 4, 0, 2}; // Indices in the Joy topic buttons array
const int stop_button_index       =                  5; // Index in the above arrays

int display_page_pins[]           = {22,23,24,25};
int display_page_light_pins[]     = { 2, 3, 4, 5};

LatchingButtonArray driveMode = LatchingButtonArray(drive_mode_count, drive_mode_pins, drive_mode_light_pins);
LatchingButtonArray displayPage = LatchingButtonArray(4, display_page_pins, display_page_light_pins);

MomentButton mouseButton = MomentButton(32,12);


Joystick joy = Joystick(A0, A1);

void drive_loop();
void mouse_loop();



bool isMouseMode = false;


void changeMessageDisplayPage(){
  page_msg.data = displayPage.getActiveButton(); 
  pagePub.publish( &page_msg );
//  Serial.print("New display page: ");
//  Serial.println(displayPage.getActiveButton());
}


void setup() {
  // Initiallize serial comms at 9600 bits per second.
  //Serial.begin(9600);
  
  joy_msg.axes=axes;
  joy_msg.axes_length=2;
  joy_msg.buttons=buttons;
  joy_msg.buttons_length=6;
  
  nh.initNode();
  nh.advertise(pagePub);
  nh.advertise(joyPub);
  nh.advertise(mousePub);

  driveMode.init(stop_button_index);
  displayPage.init();
  mouseButton.init();
}
 

void waitForMaster(){
  int idx = 0;
  long int t = millis();
  while(!nh.connected()){
    nh.spinOnce();
    delay(1);
    
    if(millis()-t > 100){
      t=millis();
      
      for(int i=0; i<6; i++) digitalWrite(drive_mode_light_pins[i], LOW);
      digitalWrite(drive_mode_light_pins[idx], HIGH);
      
      switch(idx){
        case 0: idx=1; break;
        case 1: idx=2; break;
        case 2: idx=5; break;
        case 5: idx=4; break;
        case 4: idx=3; break;
        case 3: idx=0; break;
        default: idx=0; break;
      }
    }
  }
  driveMode.updateLights();
}


void loop(){ 
  
  if(!nh.connected()) waitForMaster();
  
  
  
  joy.update();
  
  
  if (displayPage.checkForButtonChange()) changeMessageDisplayPage();
  
  if (isMouseMode){
    mouse_loop();
  } else {
    drive_loop();
  }
  nh.spinOnce();
  
  // delay in between reads for stability.
  delay(5);
}

