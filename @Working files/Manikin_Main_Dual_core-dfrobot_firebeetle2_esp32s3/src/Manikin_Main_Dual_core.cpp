#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <ESP32TimerInterrupt.h>
#include <FDC2214.h>
#include <Adafruit_NeoPixel.h>

#include "parameters.h"

// Instances
HardwareSerial SerialToC3(2);
FDC2214 capsense(FDC2214_I2C_ADDR_0);
Adafruit_NeoPixel pixels(NUMPIXELS, RGB, NEO_GRB + NEO_KHZ800);
VL53L1X lox; 
ESP32Timer timer(0);
//handshake
uint8_t HStrials = 10;
uint8_t dongleHandShake(uint8_t trail = 50){
  uint8_t i = 0;
  char buff[10] ="";
  String msg;
  uint8_t msglen = 0;
  for (i = 0; i <trail; i++){//retry for default 50times
    SerialToC3.printf("**%c**",ID);
    msglen = Serial1.available();
    if (msglen) msg = SerialToC3.readStringUntil('\n');
    else {delay(500);continue;}
    msg.trim();
    if (strcmp(msg.c_str(),"**#**")==0) break;
    // vTaskDelay(pdMS_TO_TICKS(100));
    delay(250);
  }
  if (!(i <= trail))Serial.println("Failed to handshake");
  return i;
}

// Message structure
typedef struct struct_message {
  char a[48];
} struct_message;

struct_message myTxData;
struct_message myRxData;

// Variables
unsigned long lastSend = 0;
unsigned long last_current_time = 0;
unsigned int time_stamp = 0;
bool first_touch_detected = false;
bool debounced_touch_detected = false;
bool AED_A_detected = false;
bool AED_B_detected = false;
bool CPR_mode = true;
bool CPR_flag = false;
bool peak_flag = false;


unsigned long current_time = 0;
unsigned int current_100ms = 0;
unsigned int second_counter = 0;
bool task1Flag = false;
bool vibrate_timer = false;
bool pump_timer = false;
bool aed_timer = false;

int pump_state = 0;
int vibrator_state = 0;
int aed1_state = 0; 
int aed2_state = 0; 

volatile int timer_count = 0;
int peak_count = 0;
int cpr_rate = 0;
int currentTime = 0;

int aed1_absent_count = 0;
int aed2_absent_count = 0;
const int ABSENT_THRESHOLD = 3;  
int aed1_confirm_count = 0;  
int aed2_confirm_count = 0; 

volatile bool sending_enabled = false;  
volatile bool warmup_mode = true;  

// Original TOF min/max per second
int min_dist[NUM_SECONDS];
int max_dist[NUM_SECONDS];
int current_min = OUT_OF_RANGE;
int current_max = 0;
int current_second = 0;  // 初始化为 0
unsigned long collection_start = 0;

// Rolling average filter for distance 
int max_depth_sec = 0;
int min_depth_sec = 0;

int cpr_count =0;

// Button debounce variables
unsigned long int last_button_time1 = 0;
unsigned long int last_button_time2 = 0;
bool lastState1 = HIGH;
bool lastState2 = HIGH;

// Previous states for change detection
int prev_touch_state = 0;
int prev_aed1_state = 0;
int prev_aed2_state = 0;
bool prev_state1 = false; //metal detector state
bool prev_state2 = false; //metal detector state

// New variables for UART command handling
int prev_pump_state = 0;
int prev_vibrator_state = 0;

// Global I2C mutex for synchronization
SemaphoreHandle_t i2cMutex = NULL;

// Timer ISR
// current_time: 100ms timer flag
// current_100ms: 100ms counter number - 0 to 9
// second_counter: 1 second timer flag
// vibrate_timer: vibrator timer flag (per VIB_INTERVAL)
// pump_timer: pump timer flag (per PUMP_INTERVAL)
// timer_count: game second counter for game time logging
bool IRAM_ATTR onTimer(void *timerNo) {
  current_time++;
  current_100ms++;

  task1Flag = true; 

  if (current_100ms % VIB_INTERVAL == 0) vibrate_timer = true;

  if (current_100ms % PUMP_INTERVAL == 0) pump_timer = true;

  if (current_100ms % AED_INTERVAL == 0) aed_timer = true;

  if (current_100ms % 10 == 0) {
    second_counter++;
    current_100ms = 0;
    if (timer_count < 999) timer_count++;
    else timer_count = 999;
  }
  return true;
}
// RGB
void RGB_mode(){
  if (warmup_mode && CPR_mode) { //Red for warm up mode & CPR rate mode
    pixels.setPixelColor(0, pixels.Color(150, 0, 0));
    pixels.show();
  }
  else if (warmup_mode && !CPR_mode) { //Purple for warm up mode & Counter rate mode
    pixels.setPixelColor(0, pixels.Color(150, 0, 150));
    pixels.show();    
  }
  else if (!warmup_mode && CPR_mode) { //Green for game mode & CPR rate mode
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
    pixels.show();    
  }	 
  else if (!warmup_mode && !CPR_mode) { //White for game mode & Counter mode
    pixels.setPixelColor(0, pixels.Color(150, 150, 150));
    pixels.show();    
  }	  
}

// Cap Sensor Routines
unsigned long capa[CHAN_COUNT];
unsigned long cal_cap[CHAN_COUNT];
bool touch_baseline_ready = false;

void read_cap(){
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    for (int i = 0; i < CHAN_COUNT; i++){
      capa[i]= capsense.getReading28(i);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    xSemaphoreGive(i2cMutex);
  } else {
    Serial.println("I2C mutex timeout in read_cap!");
  }
}

void recalibrate_touch_baseline() {
  Serial.println("Untouched Cap calibration starts!");
  for (int j = 0; j < BASELINE_SAMPLES; j++){
    read_cap();
    vTaskDelay(pdMS_TO_TICKS(500));
    for (int i = 0; i < CHAN_COUNT; i++){ 
      cal_cap[i] += capa[i];
    }
  }
  for (int i = 0; i < CHAN_COUNT; i++){ 
    cal_cap[i] = cal_cap[i]/BASELINE_SAMPLES;
    Serial.print("Calibrated value (CH");Serial.print(i);Serial.print("): ");
    Serial.println(cal_cap[i]);
  }
  touch_baseline_ready = true;
  Serial.println("Calibration Finished!!");
}

bool read_touch_state() {
  bool touch = false;
  read_cap();
  for (int i = 0; i < CHAN_COUNT; i++){ 
    if (cal_cap[i] > (capa[i] + TOUCH_DELTA_THRESHOLD)) {
      touch = true;
    }
  }
  return touch;
}

// ==================== AED1 金屬偵測 (連續2次 debounce) ====================
// Active HIGH
void read_AED1_metal() {
  bool detected = (digitalRead(AED1_DETECT_PIN) == HIGH);

  if (detected) {
    aed1_confirm_count++;         
    aed1_absent_count = 0;          

    if (aed1_confirm_count >= AED_DEBOUNCE_THRESHOLD) {
      aed1_state = 1;
    }
  } 
  else {
    aed1_confirm_count = 0;         
    aed1_absent_count++;          

    if (aed1_absent_count >= AED_DEBOUNCE_THRESHOLD) {
      aed1_state = 0;
    }
  }
}

void read_AED2_metal() {
  bool detected = (digitalRead(AED2_DETECT_PIN) == HIGH);

  if (detected) {
    aed2_confirm_count++;           
    aed2_absent_count = 0;          

    if (aed2_confirm_count >= AED_DEBOUNCE_THRESHOLD) {
      aed2_state = 1;
    }
  } 
  else {
    aed2_confirm_count = 0;        
    aed2_absent_count++;          

    if (aed2_absent_count >= AED_DEBOUNCE_THRESHOLD) {
      aed2_state = 0;
    }
  }
}

// Task1 
void task1(void *parameter) {
  while (true) {
    if (task1Flag) {
      task1Flag = false;
      currentTime = second_counter;

      //Read AED via metal detector
      read_AED1_metal();
      read_AED2_metal();

      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) { 
        if (lox.dataReady()) {
          uint16_t distance = lox.read(false);         
          if (distance != 0 && distance < 4000) {      
            int raw = distance + (OFFSET);        
            // if (!(current_100ms%3)) Serial.printf("raw tof: %i\n",raw);
            // Serial.printf("raw:%i\tcFlag:%s\tpFlag:%s\n",raw,CPR_flag?"TRUE":"FALSE",peak_flag?"TRUE":"FALSE");
            //V14 - change the CPR detection method: CPR_flag is triggered when distance is less than LOW_DIST
            if (raw < LOW_DIST && CPR_flag == false){
              CPR_flag = true;
            }
            //V14 - change the CPR detection method

            if (current_min == OUT_OF_RANGE) {
              current_min = raw;
            } else {
              current_min = min(current_min, raw);
            }
            current_max = max(current_max, raw);

            if (raw >= MIN_RECOIL_MM) {
              peak_flag = true;
            }

            //V13 codes
            if (CPR_flag) {
              CPR_flag = false;
              if (peak_flag) {
                cpr_count++;
              }
              peak_flag = false;
            }
            //V13 codes

            //V14 codes
            if (CPR_flag && peak_flag) {
              cpr_count++;
              CPR_flag = false;
              peak_flag = false;
            }
            //V14 codes
          }
        }
        xSemaphoreGive(i2cMutex);
      } else {
        Serial.println("I2C mutex timeout in task1!");
      }

      if (current_100ms == 1){
        first_touch_detected = read_touch_state();
        if (!first_touch_detected) {
          debounced_touch_detected = false;
        }
      }
      if (current_100ms == 4 && first_touch_detected){
        debounced_touch_detected = read_touch_state();
        if (debounced_touch_detected) {
          Serial.println("Touch Detected!");
        }
        else {
          first_touch_detected = false;
        }
      }
    }  

    int sec = currentTime - collection_start;
    if (sec > current_second && current_second < NUM_SECONDS) {  
      min_dist[current_second] = current_min;
      max_dist[current_second] = current_max;
      current_min = OUT_OF_RANGE;
      current_max = 0;
      current_second = sec;
    } else if (current_second >= NUM_SECONDS) {  
      current_second = 0;  
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Build message (CPR RATE replaces average value)
void build_and_print_message(bool is_immediate) {
  // Check for corruption
  if ((int)sending_enabled != 0 && (int)sending_enabled != 1) {
    sending_enabled = false;
    Serial.println("WARNING: sending_enabled corrupted, reset to false!");
  }

  char buffer[48];
  strcpy(buffer, "AA0121000108000207500006000105600309000001100BB");

  buffer[2] = is_immediate ? '1' : '0';
  buffer[3] = ID;

  char time_str[4];
  sprintf(time_str, "%03d", min(999, (int)timer_count));
  buffer[4] = time_str[0];
  buffer[5] = time_str[1];
  buffer[6] = time_str[2];

  // Min/Max per second 
  for (int i = 0; i < NUM_SECONDS; i++) {
    int min_dist_sec = min_dist[i];
    int max_dist_sec = max_dist[i];
    if (min_dist_sec == OUT_OF_RANGE) {
      min_dist_sec = 0;
      max_dist_sec = 0;
    } else {
      min_dist_sec = max(0, min(999, min_dist_sec));
      max_dist_sec = max(0, min(999, max_dist_sec));
    }
    min_depth_sec = (HEIGHT - max_dist_sec)*1.5;
    max_depth_sec = (HEIGHT - min_dist_sec)*1.5;
    if (abs(max_depth_sec) <= NOISE) {
      max_depth_sec = 0; //remove the noise if there is no compression
      min_depth_sec = 0; //remove the noise if there is no compression
    }  
    if ( min_depth_sec <0)  min_depth_sec = 0;
    if (max_depth_sec > MAX_DEPTH) max_depth_sec = 60; //limit the max compression depth to 60mm
    char str[4];
    sprintf(str, "%03d", min_depth_sec); //changed to min_depth_sec
    int pos = 7 + i * 6;
    buffer[pos] = str[0]; buffer[pos+1] = str[1]; buffer[pos+2] = str[2];
    sprintf(str, "%03d", max_depth_sec); //changed to max_depth_sec
    buffer[pos+3] = str[0]; buffer[pos+4] = str[1]; buffer[pos+5] = str[2];
  }
  
  // CPR mode: True - report the counts in this 5-sec period; False - report the accummulated CPR counts so far
  // ********************************
  // For debug
  int acc_cpr_count = cpr_count; 
  // Print and send
  Serial.printf("VALID CPR COUNT: %i\tVALID CPR RATE: %i\n",acc_cpr_count,cpr_rate);
  
  // ********************************



  char rate_str[4];
  sprintf(rate_str, "%03d", min(999, cpr_rate));
  buffer[37] = rate_str[0];
  buffer[38] = rate_str[1];
  buffer[39] = rate_str[2];

  // States
  int touch_state = debounced_touch_detected ? 1 : 0;
  buffer[40] = '0' + touch_state;
  buffer[41] = '0' + aed1_state;
  buffer[42] = '0' + aed2_state;
  buffer[43] = '0' + pump_state;      
  buffer[44] = '0' + vibrator_state;  
  buffer[47] = '\0';

  // Print and send
  Serial.print("Built message: ");
  Serial.println(buffer);

  memcpy(myTxData.a, buffer, sizeof(myTxData.a));
  SerialToC3.write((uint8_t*)&myTxData.a, sizeof(myTxData.a)-1);
  
  // Reset data collection per game start
  if (!is_immediate) {
    collection_start = second_counter;
    current_second = 0;
    current_min = OUT_OF_RANGE;
    current_max = 0;
  }
}

void IRAM_ATTR button_pressed() {
  unsigned long now = millis();
  if (now - last_button_time1 > DEBOUNCE1) {
    bool currentState = digitalRead(BUTTON_PIN);  // HIGH=idle, LOW=pressed
    if (currentState != lastState1) {
      lastState1 = currentState;
      last_button_time1 = now;
      if (currentState == LOW) {
        CPR_mode = !CPR_mode;
      }
    }
  }
}

// Button ISR for CPR_DET
void IRAM_ATTR cpr_pressed() {
  unsigned long now = millis();
  if (now - last_button_time2 > DEBOUNCE2) {
    bool currentState = digitalRead(CPR_DET);  // HIGH=idle, LOW=pressed
    if (currentState != lastState2) {
      lastState2 = currentState;
      last_button_time2 = now;
      if (currentState == LOW) {
        CPR_flag = true;
      }
    }
  }
}

void setup() {
  pinMode(VIB, OUTPUT); pinMode(ON_BOARD_LED, OUTPUT); pinMode(PUMP, OUTPUT); pinMode(PUMP_DIR, OUTPUT);
  pinMode(BUTTON_PIN,INPUT_PULLUP);pinMode(AED1_DETECT_PIN, INPUT_PULLDOWN); pinMode(AED2_DETECT_PIN, INPUT_PULLDOWN);
  digitalWrite(ON_BOARD_LED, LOW); digitalWrite(PUMP, LOW); digitalWrite(PUMP_DIR, LOW);

  Wire.begin(SDA, SCL);
  Wire.setClock(I2C_SPEED);

  vTaskDelay(pdMS_TO_TICKS(3000));
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(2000));

  SerialToC3.begin(UART_BAUD, SERIAL_8N1, 44, 43);
  SerialToC3.setRxBufferSize(1024); 

  uint8_t HStryno = dongleHandShake(HStrials);
  // Serial.printf("HS ends #%i/%i",HStryno,HStrials);//handshake

  // Create I2C mutex
  i2cMutex = xSemaphoreCreateMutex();
  if (i2cMutex == NULL) {
    Serial.println("Failed to create I2C mutex!");
    while(1);  // Halt if mutex fails
  }

  pixels.begin();
  pixels.setBrightness(255); 
    

  Serial.println("\nFDC2x1x test");
  vTaskDelay(pdMS_TO_TICKS(200));
  bool capOk = capsense.begin(0x3, 0x4, 0x5, false);
  if (capOk) Serial.println("Sensor OK");  
  else Serial.println("Sensor Fail");

  recalibrate_touch_baseline();

  Serial.println("Pololu VL53L1X test");
  if (!lox.init()) {
    Serial.println("Failed to boot VL53L1X");
    while(1);
  }
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  lox.setDistanceMode(VL53L1X::Short);   // Short: <1.3m       
  lox.setMeasurementTimingBudget(33000);  //us     
  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget. 
  lox.startContinuous(40);  //ms

  timer.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, onTimer);

  xTaskCreatePinnedToCore(task1, "ToF & Peak calculation and Cap Sensing", 4096, NULL, 1, NULL, 0);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_pressed, CHANGE);

  collection_start = 0;
  current_second = 0;
  current_min = OUT_OF_RANGE;
  current_max = 0;
  for (int i = 0; i < NUM_SECONDS; i++) {
    min_dist[i] = OUT_OF_RANGE;
    max_dist[i] = 0;
  }


  // RGB indicator: 
  RGB_mode();		
}

void loop() {
  RGB_mode();

  if (SerialToC3.available() > 0) {
    String input = SerialToC3.readString();
    input.trim();

    Serial.print("Received raw from C3: "); Serial.println(input);

    if (input.startsWith("AA") && input.endsWith("BB")) {
      Serial.print("Valid message: "); Serial.println(input);
      pixels.setPixelColor(0, pixels.Color(150, 150, 0)); //AMBER for receiving command from PC
      pixels.show();   

      vTaskDelay(pdMS_TO_TICKS(100));

      String cmds = input.substring(2, input.length() - 2);
      Serial.print("Extracted cmds: "); Serial.println(cmds);

      if (cmds.length() == 3) {
        if (cmds == "222" && warmup_mode) {
          build_and_print_message(true);  // 傳 type1，如你的需求
        } else {
          char control_char = cmds.charAt(0);
          Serial.print("Control char: "); Serial.println(control_char);

          if (control_char == '1') {
            warmup_mode = false;
            if (!sending_enabled) {
              timer_count = 0;  
            }
            sending_enabled = true;
            Serial.print("Game started, sending_enabled: "); Serial.println((int)sending_enabled);
            build_and_print_message(true);
          } else if (control_char == '0') {
            sending_enabled = false;
            timer_count = 0;
            warmup_mode = true;
            build_and_print_message(true);  
          }

          if ((cmds.startsWith("1") && (cmds == "100" || cmds == "111" || cmds == "101" || cmds == "110")) || (cmds.startsWith("0") && (cmds == "000" || cmds == "011" || cmds == "001" || cmds == "010"))) {  // 只在 control_char=='1' 下處理
            pump_state = cmds.charAt(1) - '0';
            vibrator_state = cmds.charAt(2) - '0';

            if (pump_state != prev_pump_state || vibrator_state != prev_vibrator_state) {
              if (sending_enabled) {
                build_and_print_message(true);
              }
              prev_pump_state = pump_state;
              prev_vibrator_state = vibrator_state;
            }
          }
        }
      RGB_mode();
      }
    } else {
      Serial.println("Invalid message received from C3");
    }
  }

  int current_touch_state = debounced_touch_detected ? 1 : 0;
  bool state_changed = (current_touch_state != prev_touch_state) ||
                       (aed1_state != prev_aed1_state) ||
                       (aed2_state != prev_aed2_state);
  //V14 - clear all the CPR flag and peak flag if time > CLEAR_PEAK
  if (current_time - last_current_time >= CLEAR_PEAK) {
    peak_flag = false;
    CPR_flag = false; //V14
    last_current_time = current_time;
  }
  if (sending_enabled && !warmup_mode && second_counter - lastSend >= 5) { //5Second counter
  pixels.setPixelColor(0, pixels.Color(0, 0, 150));
  pixels.show();
  // Serial.println(second_counter);							   
  if (current_second < NUM_SECONDS && current_min != OUT_OF_RANGE) {
    min_dist[current_second] = current_min;
    max_dist[current_second] = current_max;
  }
    if (CPR_mode) { //counts in this 5-second period
      cpr_rate = cpr_count; 
      cpr_count =0 ;
    } else { //accumulated counts so far
      cpr_rate = cpr_count;
    }
    if (state_changed) {
      pixels.setPixelColor(0, pixels.Color(0, 0, 150));
      pixels.show();
      vTaskDelay(pdMS_TO_TICKS(100));
      if (current_second < NUM_SECONDS && current_min != OUT_OF_RANGE) {
        min_dist[current_second] = current_min;
        max_dist[current_second] = current_max;
      }
      if (sending_enabled) {  
        build_and_print_message(true);
      }
      prev_touch_state = current_touch_state;
      prev_aed1_state = aed1_state;
      prev_aed2_state = aed2_state;
      RGB_mode();												 			  
    }
  
    vTaskDelay(pdMS_TO_TICKS(100));

    build_and_print_message(false);
    lastSend = second_counter;
    RGB_mode();	
  }

  if (aed_timer) {
    bool state1 = (digitalRead(AED1_DETECT_PIN) == HIGH); //active HIGH
    bool state2 = (digitalRead(AED2_DETECT_PIN) == HIGH); //active HIGH
    if (state1 == true ){
      if (prev_state1 == true) aed1_state = 1; // valid detection for two consecutive detections
      prev_state1 = true; //set it to true if it is not
    }
    else {
      if (prev_state1 == false) aed1_state = 0; // valid not_detected for two consecutive not_detections
      prev_state1 = false; //set it to false if it is not
    }
    if (state2 == true){
      if (prev_state2 == true) aed2_state = 1; // valid detection for two consecutive detections
      prev_state2 = true; //set it to true if it is not
    }
    else {
      if (prev_state2 == false) aed2_state = 0; // valid not_detected for two consecutive not_detections
      prev_state2 = false; //set it to false if it is not
    }
  }

  if (vibrate_timer) {
    if (vibrator_state == 1 && current_100ms < 3) digitalWrite(VIB, HIGH);
    else digitalWrite(VIB, LOW);
    vibrate_timer = false;
  }

  if (pump_timer) {
    if (pump_state == 1) {
      digitalWrite(PUMP, HIGH);
      digitalWrite(PUMP_DIR, !digitalRead(PUMP_DIR));
      digitalWrite(ON_BOARD_LED, !digitalRead(ON_BOARD_LED));
    }
    else {
      digitalWrite(PUMP, LOW);
      digitalWrite(ON_BOARD_LED, LOW);
    }
    pump_timer = false;
  }
}
