
class RingBufferFloat{
public:
  RingBufferFloat(int buffer_size, float initVal=0.0){
    count = buffer_size;
    arr = (float*)malloc(sizeof(float)*count);
    idx = 0;
    for (int i=0; i<count; i++) {
      arr[i] = initVal/count;
      avg += arr[i];
    }
  }

  void append(float val){
    avg -= arr[idx];
    arr[idx] = val/count;
    avg += arr[idx];
    idx = (idx+1)%count;
  }
  int read(){ return avg; }
  
private:
  int count;
  int idx;
  float avg;
  bool isValid;
  float* arr;
}; 




class LatchingButtonArray{
public:
  LatchingButtonArray(int tcount, int* sigpins, int* lightpins){
    count = tcount;
    sigpin_arr=sigpins;
    lightpin_arr=lightpins;
    active_button = 0;
  }

  void init(){
    for (int i =0; i < count; i++){
      pinMode(sigpin_arr[i], INPUT);
      pinMode(lightpin_arr[i], OUTPUT);
    }
    updateLights();
  }

  bool checkForButtonChange(){
    bool hasChanged = false;
    for (int i = count-1; i >= 0; i--){ // Run loop in reverse to go from lowest priority to highest
      if (i != active_button && digitalRead(lightpin_arr[i]) == HIGH){
        active_button = i;
        hasChanged = true;
      }
    }
    if (hasChanged) updateLights();
    return hasChanged;
  }

  void forceButtonChange(int idx){
    if (idx >= count) return;
    active_button = idx;
    updateLights();
  }

  int getActiveButton(){ return active_button; }
  
  void updateLights(){
    for (int i =0; i < count; i++){
      digitalWrite(lightpin_arr[i], LOW);
    }
    digitalWrite(lightpin_arr[active_button], HIGH);
  }
  
private:
  int count;
  int* sigpin_arr;
  int* lightpin_arr;
  int active_button;
}; 




// Class that implements a state machine to debounce a virtual latching button
class LatchingButton{
public:
  LatchingButton(int sigpin, int lightpin){
    signal_pin=sigpin;
    light_pin=lightpin;
    state = off;
  }
  
  void init(){
    pinMode(signal_pin, INPUT);
    pinMode(light_pin, OUTPUT);
    updateLight();
  }

  bool checkForButtonChange(){
    bool hasChanged = false;
    int pinVal = digitalRead(signal_pin);
    if      (state == off      && pinVal == HIGH) {state = presson;  hasChanged = true;}
    else if (state == presson  && pinVal == LOW)  {state = on;}
    else if (state == on       && pinVal == HIGH) {state = pressoff; hasChanged = true;}
    else if (state == pressoff && pinVal == LOW)  {state = off;}
    if (hasChanged) updateLight();
    return hasChanged;
  }

  bool read() {
    if (state == on || state == presson) return true;
    else return false;
  }
  
private:
  enum button_state {off, presson, on, pressoff} state;
  int signal_pin;
  int light_pin;
  void updateLight(){
    if (state == on || state == presson) digitalWrite(0, HIGH);
    else digitalWrite(0, LOW);
  }
}; 




// Class that implements a state machine to debounce a virtual latching button
class MomentButton{
public:
  MomentButton(int sigpin, int lightpin){
    signal_pin=sigpin;
    light_pin=lightpin;
    state = LOW;
  }
  
  void init(){
    pinMode(signal_pin, INPUT);
    pinMode(light_pin, OUTPUT);
  }

  bool checkForButtonChange(){
    bool hasChanged = false;
    int pinVal = digitalRead(signal_pin);
    if (pinVal != state) {
      state = pinVal;
      digitalWrite(light_pin, state);
      hasChanged = true;
    }
    return hasChanged;
  }

  bool read() {
    if (state == HIGH) return true;
    else return false;
  }
  
private:
  int state;
  int signal_pin;
  int light_pin;
}; 



// Class that implements a state machine to debounce a virtual latching button

const int joy_avg_length = 6;
const double joyXDeadLimit = 10/512.0; // Post-mapping
const double joyYDeadLimit = 10/512.0; // Post-mapping

class Joystick{
public:
  Joystick(int xpin, int ypin){
    x_pin = xpin;
    y_pin = ypin;
    isCalibrating = false;
    setCalibration(0, 512, 1023, 0, 512, 1023);
  }

  void setCalibration(int xMin, int xCen, int xMax, int yMin, int yCen, int yMax){
    cal.x_min = xMin;
    cal.x_center = xCen;
    cal.x_max = xMax;
    
    cal.y_min = yMin;
    cal.y_center = yCen;
    cal.y_max = yMax;
    
    updateHalfRange();
  }

  // Clears the calibration and sets the center point to the current value
  // Averaged over ~25ms
  // TODO: Use temporary struct for calibration
  void clearCalibration(){
    int joyXRaw = 0, joyYRaw = 0;
    for (int i = 0; i < 8; i++){
      joyXRaw += analogRead(x_pin);
      joyYRaw += analogRead(y_pin);
      delay(3);
    }
    joyXRaw /= 8;
    joyYRaw /= 8;
    setCalibration(joyXRaw, joyXRaw, joyXRaw, joyYRaw, joyYRaw, joyYRaw);
    isCalibrating = true;
  }

  // Extends the current calibration values by the 
  void updateCalibration(){
    int joyXRaw = analogRead(x_pin);
    int joyYRaw = analogRead(y_pin);

    if (joyXRaw > cal.x_max) cal.x_max = joyXRaw;
    if (joyXRaw < cal.x_min) cal.x_min = joyXRaw;
    if (joyYRaw > cal.y_max) cal.y_max = joyYRaw;
    if (joyYRaw < cal.y_min) cal.y_min = joyYRaw;
  }

  void saveCalibration(){
    isCalibrating = false;
    updateHalfRange();
    cal.checksum = Fletcher16((uint8_t*)(&cal), 12);

    // TODO: Save to EEPROM
Serial.print("X Min = ");
Serial.println(cal.x_min);
Serial.print("X Ctr = ");
Serial.println(cal.x_center);
Serial.print("X Max = ");
Serial.println(cal.x_max);

Serial.print("Y Min = ");
Serial.println(cal.y_min);
Serial.print("Y Ctr = ");
Serial.println(cal.y_center);
Serial.print("Y Max = ");
Serial.println(cal.y_max);

Serial.print("Checksum = ");
Serial.println( cal.checksum );


  }
  
  void update(){
    if (isCalibrating) {
      joyXBuffered.append(0.0);
      joyYBuffered.append(0.0);
      return;
    }
    
    int joyXRaw = analogRead(x_pin);
    int joyYRaw = analogRead(y_pin);
    
    // map the values (x values. Joystick below neutral position)
    double joyX = (joyXRaw-cal.x_center)/x_half_range;
    double joyY = (joyYRaw-cal.y_center)/y_half_range; // reverse?
    
    // Zero joystick if in dead zone
    if (abs(joyX) < joyXDeadLimit) joyX = 0;
    if (abs(joyY) < joyYDeadLimit) joyY = 0;
  
    // Run averaging
    joyXBuffered.append(joyX);
    joyYBuffered.append(joyY);

    
// print the sensor value: (For Debugging)
Serial.print("X raw value = ");
Serial.print(joyXRaw);
Serial.print("\t mapped value = ");
Serial.println(joyX);
Serial.print("\t averaged value = ");
Serial.println(joyXBuffered.read());
Serial.print("Y raw value = ");
Serial.print(joyYRaw);
Serial.print("\t mapped value = ");
Serial.println(joyY);
Serial.print("\t averaged value = ");
Serial.println(joyYBuffered.read());
  }

  bool getX() {
    return joyXBuffered.read();
  }
  bool getY() {
    return joyYBuffered.read();
  }
  
private:
  RingBufferFloat joyXBuffered = RingBufferFloat(joy_avg_length);
  RingBufferFloat joyYBuffered = RingBufferFloat(joy_avg_length);
  int x_pin;
  int y_pin;
  
  float x_half_range;
    
  float y_half_range;

  bool isCalibrating;

  
  struct Calibration {
    int16_t x_min;
    int16_t x_center;
    int16_t x_max;
      
    int16_t y_min;
    int16_t y_center;
    int16_t y_max; 
    
    int16_t checksum; 
  } cal;
  
  void updateHalfRange(){
    x_half_range = ( (cal.x_center-cal.x_min) > (cal.x_max-cal.x_center) ) ? (cal.x_center-cal.x_min) : (cal.x_max-cal.x_center);
    y_half_range = ( (cal.y_center-cal.y_min) > (cal.y_max-cal.y_center) ) ? (cal.y_center-cal.y_min) : (cal.y_max-cal.y_center);
  }

// https://en.wikipedia.org/wiki/Fletcher%27s_checksum#Straightforward
  uint16_t Fletcher16( uint8_t *data, int count )
  {
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    int index;
    
    for( index = 0; index < count; ++index )
    {
      sum1 = (sum1 + data[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
    }
    
    return (sum2 << 8) | sum1;
  }
}; 

