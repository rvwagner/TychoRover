
const int mode_u1 = drive_mode_pins[0];
const int mode_u2 = drive_mode_pins[1];
const int mode_u3 = drive_mode_pins[2];
const int mode_l1 = drive_mode_pins[3];
const int mode_l2 = drive_mode_pins[4];
const int mode_l3 = drive_mode_pins[5];
byte flashingButtons = B00111111;

bool mouseDown = true;


void sendMouseMessage(){
  
  Serial.print("Joystick Position: ");
  Serial.print(joy.getX());
  Serial.print(joy.getY());
  if(mouseDown) Serial.println(" (button down)");
}




void flashModeButtons(int Hz = 1){
  static long lastTime = 0;
  static int brightness = 0;
  static int sign = 1;

  int newTime = millis();
  int dt = newTime - lastTime;

  // dt ms
  // 511 units/Cycle
  // Hz / 1000 Cycle/ms
  int change = dt * 511 * Hz / 1000;
  if (change < 2 ) return;
  
  lastTime = newTime;
  brightness += sign * change;
  if (brightness >= 255) {
    brightness = 255; sign = -1;
  } else if (brightness <= 0) {
    brightness = 0; sign = 1;
  }
  
  for (int i = 0; i < drive_mode_count; i++){
    if (flashingButtons & (1<<i))
      analogWrite(drive_mode_light_pins[i], brightness);
    else
      digitalWrite(drive_mode_light_pins[i], LOW);
  }
}

void exitMouseMode(){
  Serial.println("Exiting mouse mode...");
  isMouseMode = false;
  flashingButtons = B00111111;
  driveMode.updateLights();
}


void calibrate_joystick(){
  // Wait for buttons to be released
  while (digitalRead(mode_u1) == HIGH || digitalRead(mode_l1) == HIGH) {
#ifdef ROS
   nh.spinOnce();
#endif
    flashModeButtons(2);
    delay(1);
  }
  
  flashingButtons = B00100100; // flash only left two buttons
  joy.clearCalibration();
  
  while (digitalRead(mode_u1) != HIGH && digitalRead(mode_l1) != HIGH && digitalRead(mode_u3) != HIGH) {
    joy.updateCalibration();
#ifdef ROS
   nh.spinOnce();
#endif
    flashModeButtons(2);
    delay(1);
  }
  
  flashingButtons = B00111111; // flash all buttons
  joy.saveCalibration();
}

void mouse_loop(){
  static long lastSend = 0;
  bool valuesChanged = false;
  int u1 = (digitalRead(mode_u1) == HIGH) ? 1 : 0;
  int u2 = (digitalRead(mode_u2) == HIGH) ? 1 : 0;
  int u3 = (digitalRead(mode_u3) == HIGH) ? 1 : 0;
  int l1 = (digitalRead(mode_l1) == HIGH) ? 1 : 0;
  int l2 = (digitalRead(mode_l2) == HIGH) ? 1 : 0;
  int l3 = (digitalRead(mode_l3) == HIGH) ? 1 : 0;
  int buttonDownCount = (u1 + u2 + u3 + l1 + l2 + l3);
  
  if (buttonDownCount == 2 && u1 && l1){
    calibrate_joystick();
  } else if (buttonDownCount == 2 && u2 && l2){
  } else if (buttonDownCount == 2 && u3 && l3){
  } else if (!mouseDown && buttonDownCount == 1){
    // Exactly one button: mouseclick
    valuesChanged = true;
    mouseDown = true;
  } else if (mouseDown && buttonDownCount == 0){
    // No buttons: end mouseclick
    valuesChanged = true;
    mouseDown = false;
  }

  // Exit mouse mode if needed and if the controls are safe
  if (joy.getX() == 0 && joy.getY() == 0 && buttonDownCount == 0) {
    if (mouseButton.checkForButtonChange()) {
      exitMouseMode();
    }
  }
  
  flashModeButtons();
  
  // If it's been a while or the joystick has changed, send a packet
  if (valuesChanged || (lastSend - millis()) > minSendInterval){
    sendMouseMessage();
    lastSend = millis();
  }

}
