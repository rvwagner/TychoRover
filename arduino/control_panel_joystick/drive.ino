

void sendJoyMessage(){
#ifdef ROS
  joy_msg.header.stamp = nh.now();
  joy_msg.axes[0] = joy.getX();
  joy_msg.axes[1] = joy.getY();
  joyPub.publish( &joy_msg );
#endif
}
void setJoyButton(int idx, int val){
#ifdef ROS
  joy_msg.buttons[idx] = val;
#endif
}

void changeMessageDriveMode(){
  for (int i = 0; i < drive_mode_count; i++){
     setJoyButton(drive_mode_button_ids[i], 0);
  }
  setJoyButton(drive_mode_button_ids[driveMode.getActiveButton()], 1);  
}

  
void drive_loop() {
  static long lastSend = 0;
  bool valuesChanged = false;
  
  // If the joystick is centered, allow drive mode switching
  if (joy.getX() == 0 && joy.getY() == 0) {
    if (driveMode.checkForButtonChange()) {
      changeMessageDriveMode();
      valuesChanged = true;
    }
    if (mouseButton.checkForButtonChange()) {
      driveMode.forceButtonChange(stop_button_index);
      isMouseMode = true;
      valuesChanged = true;
    }
  }

  
  // Check other buttons

  // If it's been a while or the joystick has changed, send a packet
  if (valuesChanged || (lastSend - millis()) > minSendInterval){
    sendJoyMessage();
    lastSend = millis();
  }
  
  
  // delay in between reads for stability.
  delay(3);
}
