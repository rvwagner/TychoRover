
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
    if (avg < 0.0001 && avg > -0.0001) avg = 0; // This is < 1 / (1023*6)
  }
  float read(){ return avg; }
  
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

  void init(int starting_button=0){
    for (int i =0; i < count; i++){
      pinMode(sigpin_arr[i], INPUT_PULLUP);
      pinMode(lightpin_arr[i], OUTPUT);
    }
    active_button = starting_button;
    updateLights();
  }

  bool checkForButtonChange(){
    bool hasChanged = false;
    for (int i = count-1; i >= 0; i--){ // Run loop in reverse to go from lowest priority to highest
      if (i != active_button && digitalRead(sigpin_arr[i]) == LOW){
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
    pinMode(signal_pin, INPUT_PULLUP);
    pinMode(light_pin, OUTPUT);
    updateLight();
  }

  bool checkForButtonChange(){
    bool hasChanged = false;
    int pinVal = digitalRead(signal_pin);
    if      (state == off      && pinVal == LOW) {state = presson;  hasChanged = true;}
    else if (state == presson  && pinVal == HIGH)  {state = on;}
    else if (state == on       && pinVal == LOW) {state = pressoff; hasChanged = true;}
    else if (state == pressoff && pinVal == HIGH)  {state = off;}
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
    state = HIGH;
  }
  
  void init(){
    pinMode(signal_pin, INPUT_PULLUP);
    pinMode(light_pin, OUTPUT);
  }

  bool checkForButtonChange(){
    bool hasChanged = false;
    int pinVal = digitalRead(signal_pin);
    if (pinVal != state) {
      state = pinVal;
      digitalWrite(light_pin, !state);
      hasChanged = true;
    }
    return hasChanged;
  }

  bool read() {
    if (state == LOW) return true;
    else return false;
  }
  
private:
  int state;
  int signal_pin;
  int light_pin;
}; 



// Class that implements a state machine to debounce a virtual latching button

const int joy_avg_length = 6;

class Joystick{
public:
  Joystick(int xpin, int ypin){
    x_pin = xpin;
    y_pin = ypin;
    isCalibrating = false;
    setCalibration(0, 500, 540, 1023, 0, 470, 520, 1023);
  }

  void setCalibration(int xMin, int xMinDZ, int xMaxDZ, int xMax, int yMin, int yMinDZ, int yMaxDZ, int yMax){
    cal.x_min = xMin;
    cal.x_min_dead = xMinDZ;
    cal.x_max_dead = xMaxDZ;
    cal.x_max = xMax;
    
    cal.y_min = yMin;
    cal.y_min_dead = yMinDZ;
    cal.y_max_dead = yMaxDZ;
    cal.y_max = yMax;
    
    updateHalfRange();
  }

  // Clears the calibration and sets the center point to the current value
  // Averaged over ~25ms
  // TODO: Use temporary struct for calibration
  // TODO: Add ability to calibrate dead zone, somehow
  void clearCalibration(){
    int joyXRaw = 0, joyYRaw = 0;
    for (int i = 0; i < 8; i++){
      joyXRaw += analogRead(x_pin);
      joyYRaw += analogRead(y_pin);
      delay(3);
    }
    joyXRaw /= 8;
    joyYRaw /= 8;
    setCalibration(joyXRaw, cal.x_min_dead, cal.x_max_dead, joyXRaw, joyYRaw, cal.y_min_dead, cal.y_max_dead, joyYRaw);
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
    cal.checksum = Fletcher16((uint8_t*)(&cal), 20);

    // TODO: Save to EEPROM
  }
  
  void update(){
    if (isCalibrating) {
      joyXBuffered.append(0.0);
      joyYBuffered.append(0.0);
      return;
    }
    
    int joyXRaw = analogRead(x_pin);
    int joyYRaw = analogRead(y_pin);
    double joyX, joyY;
    
    if (joyXRaw <= cal.x_min_dead){
      joyX = (double)(joyXRaw - cal.x_min_dead)/cal.x_low_range;
    } else if (joyXRaw >= cal.x_max_dead){
      joyX = (double)(joyXRaw - cal.x_max_dead)/cal.x_high_range;
    } else {
      joyX = 0.0;
    }
    
    if (joyYRaw <= cal.y_min_dead){
      joyY = (double)(joyYRaw - cal.y_min_dead)/cal.y_low_range;
    } else if (joyYRaw >= cal.y_max_dead){
      joyY = (double)(joyYRaw - cal.y_max_dead)/cal.y_high_range;
    } else {
      joyY = 0.0;
    }
  
    // Run averaging
    joyXBuffered.append(joyX);
    joyYBuffered.append(joyY);

    /*
// print the sensor value: (For Debugging)
Serial.print("X raw value = ");
Serial.print(joyXRaw);
Serial.print("\t mapped value = ");
Serial.print(joyX);
Serial.print("\t averaged value = ");
Serial.println(joyXBuffered.read());
Serial.print("Y raw value = ");
Serial.print(joyYRaw);
Serial.print("\t mapped value = ");
Serial.print(joyY);
Serial.print("\t averaged value = ");
Serial.println(joyYBuffered.read());
*/
  }

  float getX() {
    return joyXBuffered.read();
  }
  float getY() {
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
    int16_t x_low_range;
    int16_t x_min_dead;
    int16_t x_max_dead;
    int16_t x_high_range;
    int16_t x_max;
      
    int16_t y_min;
    int16_t y_low_range;
    int16_t y_min_dead;
    int16_t y_max_dead;
    int16_t y_high_range;
    int16_t y_max; 
    
    int16_t checksum; 
  } cal;
  
  void updateHalfRange(){
    cal.x_low_range  = cal.x_min_dead - cal.x_min;
    cal.x_high_range = cal.x_max - cal.x_max_dead;
    cal.y_low_range  = cal.y_min_dead - cal.y_min;
    cal.y_high_range = cal.y_max - cal.y_max_dead;
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

