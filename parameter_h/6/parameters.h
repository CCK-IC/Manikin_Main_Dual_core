#define ID '6'
#define UART_BAUD 115200
#define SCL 2
#define SDA 1
#define I2C_SPEED 200000 

#define CHAN_COUNT 2
#define TOUCH_DELTA_THRESHOLD 150000
#define BASELINE_SAMPLES 5

#define BUTTON_PIN 11
#define CPR_DET 47
#define DEBOUNCE1 100  // in ms
#define DEBOUNCE2 30  // in ms

#define VIB 14
#define ON_BOARD_LED 21
#define PUMP 4
#define PUMP_DIR 5
#define RGB 10
#define NUMPIXELS 1
#define VIB_INTERVAL 5 //x 100ms
#define PUMP_INTERVAL 10 //x 100ms

#define OUT_OF_RANGE INT_MAX
#define NUM_SECONDS 5
#define READINGS_PER_SECOND 10
#define TOTAL_READINGS (NUM_SECONDS * READINGS_PER_SECOND)  // 50
#define READ_INTERVAL_MS (1000 / READINGS_PER_SECOND)

#define TIMER0_INTERVAL_MS 100  
#define DISTANCE_THRESHOLD_MM 150

#define NORMAL_MIN_MM 100  
#define NORMAL_MAX_MM 150 
#define AVG_SAMPLES 5

#define HEIGHT 78 //idle height of the manikin
#define MAX_DEPTH 60 //maximum compression
#define NOISE 15 //heigth detection noise with no compression
#define OFFSET -5 //Idle distance = 80mm
#define MIN_RECOIL_MM 70 
#define CLEAR_PEAK 300 //change in V14
#define DEBOUNCE_PEAK_MS 75 //shortest count possible, change in V16
#define LOW_DIST 49 //Added in V14

#define AED1_DETECT_PIN 38
#define AED2_DETECT_PIN 8
#define AED_DEBOUNCE_THRESHOLD 2   