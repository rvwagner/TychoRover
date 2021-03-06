

void sendJoyMessage(){
  joy_msg.header.stamp = nh.now();
  joy_msg.axes[0] = joy.getX();
  joy_msg.axes[1] = joy.getY();
  for (int i = 0; i < drive_mode_count; i++){
    joy_msg.buttons[drive_mode_button_ids[i]] = 0;
  }
  joy_msg.buttons[drive_mode_button_ids[driveMode.getActiveButton()]] = 1;
  joyPub.publish( &joy_msg );
}



  
void drive_loop() {
  static unsigned long lastSend = 0;
  bool valuesChanged = false;
  
  // FIXME: QUICK HACK TO REPURPOSE MOUSE BUTTON FOR EVENT FLAGGING
  if ( mouseButton.checkForButtonChange() ) {
    logFlagPub.publish( &log_flag_msg );
  }
  // If the joystick is centered, allow drive mode switching
  if (joy.getX() == 0 && joy.getY() == 0) {
    valuesChanged = driveMode.checkForButtonChange() || valuesChanged;

//    if ( mouseButton.checkForButtonChange() && mouseButton.read()) {
//      driveMode.forceButtonChange(stop_button_index);
//      isMouseMode = true;
//      valuesChanged = true;
//    }
  }

  // If it's been a while or the joystick has changed, send a packet
  if (valuesChanged || (millis() - lastSend) > minSendInterval){
    sendJoyMessage();
    lastSend = millis();
  }
  
  
}
