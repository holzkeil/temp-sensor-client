//////////////
// Includes //
//////////////

#include <OneWire.h>
#include <DallasTemperature.h>
#include <NRFLite.h>
#include <U8g2lib.h>
#include <EEPROM.h>
#include <SPI.h>

///////////////////////////////////
// CONSTANTS                     //
// can be adjusted to your needs //
///////////////////////////////////

#define TASK_INTERVAL_BUTTON 50
#define TASK_INTERVAL_TEMP 1000
#define TASK_INTERVAL_RADIO 10000
#define TASK_INTERVAL_EEPROM 30000
#define TASK_INTERVAL_TEMP_EEPROM_ADDRESS 0

#define TEMP_SENSOR_PIN 4
#define TEMP_SENSOR_RESOLUTION 12  // 12: highest, 0.0625 to 9: lowest, 0.5
#define TEMP_SENSOR_STRING_DECIMALS 2

#define BUTTON_PIN 7
#define BUTTON_LONG_PRESS_THRESHOLD 1500

#define LED_PIN 6

#define RADIO_ID_SOURCE 1
#define RADIO_ID_DESTINATION 0
#define RADIO_PIN_CE 2
#define RADIO_PIN_CSN 3
// By default, 'init' configures the radio to use a 2MBPS bitrate on channel 100 (channels 0-125 are valid).
// Both the RX and TX radios must have the same bitrate and channel to communicate with each other.
#define RADIO_SPEED NRFLite::BITRATE2MBPS // NRFLite::BITRATE250KBPS NRFLite::BITRATE1MBPS NRFLite::BITRATE2MBPS
#define RADIO_CHANNEL 100 // 0 - 125

// totalDuration        128s, 256s, 640s,21:20m,   32m,42:40m,   64m, 2:08h,  4:16h,  6:24h, 10:40h, 21:20h,     1d,    1.5d,      2d,      3d,      7d,     14d
//singleDuration          1s,   2s,   5s,   10s,   15s,   20s,   30s,    1m,     2m,     3m,     5m,    10m, 11:15m, 16.875m,   22.5m,  33.75m,  78.75m,  157.5m
#define INTERVAL_VALUES 1000, 2000, 5000, 10000, 15000, 20000, 30000, 60000, 120000, 180000, 300000, 600000, 675000, 1012500, 1350000, 2025000, 4725000, 9450000
#define INTERVAL_VALUE_COUNT 18
#define INTERVAL_MAX_STEP_MILLIS 5000

#define DISPLAY_PIN_CHIP_SELECT_DS 10
#define DISPLAY_PIN_DATA_COMMAND 9
#define DISPLAY_PIN_RESET 8

#define DISPLAY_CONSTRUCTOR U8G2_SSD1306_128X64_VCOMH0_1_4W_HW_SPI
#define DISPLAY_ROTATION U8G2_R0
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_HEADER 16
#define DISPLAY_FONT u8g2_font_5x8_tf
//#define DISPLAY_FONT u8g2_font_5x7_tf
//#define DISPLAY_FONT u8g2_font_4x6_tf

#define SERIAL_SPEED 115200

#define TASK_DEBUG_BUTTON false

///////////////////////////////////////////////////
// TODO
///////////////////////////////////////////////////

///////////////////////////////////////////////////
// Task Class that allows pseudo parallelization //
// call .run() of all your Tasks in main()       //
///////////////////////////////////////////////////

class Task {
    uint32_t previousMillis;
    uint32_t currentMillis;
    void (*functionPointer)();
    String functionName;

  public:
    uint32_t interval;

    Task(void (*function)(), uint32_t new_interval, String name) {
      functionPointer = function;
      interval = new_interval;
      previousMillis = 0;
      functionName = name;
      return;
    }

    reset(){
      previousMillis = millis();
    }
    
    run() {
      currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        if (functionName != "butn" || TASK_DEBUG_BUTTON){
          Serial.println(functionName + ": " + String(currentMillis - previousMillis));
        }
        previousMillis = currentMillis;
        functionPointer();
      }
    }
};

//////////////////////////////////////////
// ValueGraphArray stores measured data //
// and calculates optimal graph         //
//////////////////////////////////////////

class ValueGraphArray {
  private:
    bool initDone = false;
    float deltaPerPixel;
    
  public:
    float minimum;
    float maximum;
    uint8_t currentPosition = 0;
    float values[DISPLAY_WIDTH] = {};
    bool mustDraw = true;
    
    uint32_t stepCount = 1;
    uint32_t currentStep = 0;
    float currentValue = 0;

  void add(float value){
    init(value);

    currentValue = (currentValue * currentStep++ + value) / currentStep;
    values[currentPosition] = currentValue;
    
    if (currentStep == stepCount){
      currentPosition = ++currentPosition % DISPLAY_WIDTH;
      values[currentPosition] = value;
      currentStep = 0;
      currentValue = 0;
    }
    
    minimum = values[0];
    maximum = values[0];
    for (uint8_t i=1; i < DISPLAY_WIDTH; i++){
      if (values[i] < minimum) minimum = values[i];
      if (values[i] > maximum) maximum = values[i];
    }
    
    deltaPerPixel = maximum == minimum ? 1 : (maximum - minimum) / (DISPLAY_HEIGHT - DISPLAY_HEADER);
    Serial.println("DDP " + String(deltaPerPixel, 6));
    mustDraw = true;
  }

  void change_interval(uint32_t steps){
    stepCount = steps;
    if (currentStep > 0){
      currentPosition = ++currentPosition % DISPLAY_WIDTH;
      values[currentPosition] = values[(currentPosition + DISPLAY_WIDTH - 1) % DISPLAY_WIDTH];
      currentStep = 0;
      currentValue = 0;
    }
    mustDraw = true;
  }

  void init(float value){
    if (!initDone){
      for (uint8_t i=0; i < DISPLAY_WIDTH; i++){
        values[i] = value;
      }
      initDone = true;
    }
  }
  
  uint8_t get_y_position(uint8_t position){
    return DISPLAY_HEIGHT - max(1, (values[position] - minimum) / deltaPerPixel);
  }
};

///////////////////////////////////
// NRF2401L Struct and Variables //
// max 32 bytes RadioData        //
///////////////////////////////////

struct RadioData
{
    uint8_t sourceRadioId;
    float temperature;
    uint32_t intervalRadio;
    uint32_t intervalReadTempTask;
    uint32_t intervalReadTempGlobal;    
};

NRFLite radio;
RadioData radioData;

/////////////////////////////////////////////
// Read Temperature Variables and Function //
/////////////////////////////////////////////

OneWire oneWire(TEMP_SENSOR_PIN); 
DallasTemperature sensors(&oneWire);
ValueGraphArray temperatureArray = ValueGraphArray();
float temperature;

void readTemperature(void){
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);
  temperatureArray.add(temperature);
  Serial.println("T " + String(temperature, 4));
}

Task taskReadTemperature = Task(*readTemperature, TASK_INTERVAL_TEMP, "temp");

//////////////////////////////////////////////
// Read Button Press Variables and Function //
//////////////////////////////////////////////

bool buttonNewState = true;
bool buttonOldState = true;
bool buttonShortPress = false;
bool buttonLongPress = false;
uint32_t buttonTimer = 0;
uint32_t buttonCurrentMillis = 0;

void readButton() {
  buttonNewState = digitalRead(BUTTON_PIN);
  if (buttonNewState != buttonOldState) {
    if (buttonNewState == LOW) {
      buttonTimer = millis();
    }
    else {
      buttonCurrentMillis = millis();
      if (buttonCurrentMillis - buttonTimer >= BUTTON_LONG_PRESS_THRESHOLD) {
        buttonLongPress = true;
      }
      else {
        buttonShortPress = true;
      }
    }
  }
  buttonOldState = buttonNewState;
}

Task taskReadButton = Task(*readButton, TASK_INTERVAL_BUTTON, "butn");

//////////////////////////////////////////
// Update EEPROM Variables and Function //
//////////////////////////////////////////

const uint32_t intervalValues[INTERVAL_VALUE_COUNT] = {INTERVAL_VALUES};
int8_t intervalPointer;
uint32_t interval;

uint32_t oldIntervalPointer = intervalPointer;

void updateEeprom() {
  if (oldIntervalPointer != intervalPointer){
    Serial.println("O " + String(oldIntervalPointer) + " -> N " + String(intervalPointer));
    EEPROM.update(TASK_INTERVAL_TEMP_EEPROM_ADDRESS, oldIntervalPointer = intervalPointer);
  }
}

Task taskUpdateEeprom = Task(*updateEeprom, TASK_INTERVAL_EEPROM, "eprm");

////////////////////////////////////////////
// Change Interval Variables and Function //
////////////////////////////////////////////

void applyIntervalChanges(){
    interval = intervalValues[intervalPointer];
    Serial.println("I" + String(interval) + " IP" + String(intervalPointer));
    temperatureArray.change_interval(interval > INTERVAL_MAX_STEP_MILLIS ? interval / INTERVAL_MAX_STEP_MILLIS : 1);
    taskReadTemperature.interval = min(interval, INTERVAL_MAX_STEP_MILLIS);
    taskReadTemperature.reset();
    taskUpdateEeprom.reset();
}

///////////////////////////////////////////
// Send RadioData Variables and Function //
///////////////////////////////////////////

void sendRadioData(){
    radioData.intervalReadTempGlobal = interval;
    radioData.temperature = temperature;
    radioData.intervalReadTempTask = taskReadTemperature.interval;
    digitalWrite(LED_PIN, !radio.send(RADIO_ID_DESTINATION, &radioData, sizeof(radioData)));
}

Task taskSendRadioData = Task(*sendRadioData, TASK_INTERVAL_RADIO, "rdio");

//////////////////////////////////
// Main Variables and Functions //
//////////////////////////////////

String intervalToString(uint32_t interval){
  uint8_t value = 0;
  uint8_t decimal;
  String unit;

  if (interval < 60000){
    value = int(interval / 1000);
    decimal = 0;
    unit = F("s"); 
  }
  else if (interval < 3600000){
    value = int(interval / 60000);
    decimal = (interval - value * 60000) / 1000;
    unit = F("m"); 
  }
  else if (interval < 86400000){
    value = int(interval / 3600000);
    decimal = (interval - value * 3600000) / 60000;
    unit = F("h"); 
  }
  else{
    value = int(interval / 86400000);
    decimal = (interval - value * 86400000) / 3600000;
    unit = F("d"); 
  }
  String target = String(value);
  target += String(decimal);
  return String(value) + ((decimal > 0) ? (":" + String(decimal)) : "") + unit;
}

DISPLAY_CONSTRUCTOR display(DISPLAY_ROTATION, DISPLAY_PIN_CHIP_SELECT_DS, DISPLAY_PIN_DATA_COMMAND, DISPLAY_PIN_RESET);

void setup() {
  Serial.begin(SERIAL_SPEED);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT); 

  if (!radio.init(RADIO_ID_SOURCE, RADIO_PIN_CE, RADIO_PIN_CSN, RADIO_SPEED, RADIO_CHANNEL)){
    digitalWrite(LED_PIN, HIGH);
    while (1);
  }  
  radioData.sourceRadioId = RADIO_ID_SOURCE;
  radioData.intervalRadio = TASK_INTERVAL_RADIO;

  Serial.println("RDS " + String(sizeof(RadioData)));
  
  sensors.begin();
  sensors.setResolution(TEMP_SENSOR_RESOLUTION);
  
  display.begin();
  display.setFont(DISPLAY_FONT);
  display.setColorIndex(1);

  EEPROM.get(TASK_INTERVAL_TEMP_EEPROM_ADDRESS, intervalPointer);
  intervalPointer %= INTERVAL_VALUE_COUNT;
  applyIntervalChanges();
 
  readTemperature();
}

void loop() {
  taskSendRadioData.run();
  taskReadTemperature.run();
  taskUpdateEeprom.run();
  taskReadButton.run();
  
  if (buttonShortPress || buttonLongPress){
    intervalPointer = ++intervalPointer % INTERVAL_VALUE_COUNT;
    applyIntervalChanges();
    buttonShortPress = false;
    buttonLongPress = false;
  }

  if (temperatureArray.mustDraw){
    display.firstPage();
    do {
      display.setCursor(0, 6);
      String upperRow = "H " + String(temperatureArray.maximum, TEMP_SENSOR_STRING_DECIMALS);
      upperRow += " " + String(temperatureArray.values[(temperatureArray.currentPosition+DISPLAY_WIDTH-1)%DISPLAY_WIDTH], TEMP_SENSOR_STRING_DECIMALS); 
      upperRow += " " + String(temperatureArray.currentValue, TEMP_SENSOR_STRING_DECIMALS);
      upperRow += " " + String(temperature, TEMP_SENSOR_STRING_DECIMALS);
      display.print(upperRow); 
      
      display.setCursor(0, 14);
      String lowerRow = "L " + String(temperatureArray.minimum, TEMP_SENSOR_STRING_DECIMALS);
      if (temperatureArray.stepCount > 1){
        lowerRow += " " + String(temperatureArray.currentStep) + "/" + String(temperatureArray.stepCount); 
      }
      lowerRow += " " + intervalToString(interval) + " " + intervalToString(interval*DISPLAY_WIDTH);
      display.print(lowerRow);
      
      for (uint8_t i=0; i < DISPLAY_WIDTH; i++){
        uint8_t y_position = temperatureArray.get_y_position((DISPLAY_WIDTH - i + temperatureArray.currentPosition)%DISPLAY_WIDTH);
        display.drawPixel(DISPLAY_WIDTH - 1 - i, y_position);
      }
    } while (display.nextPage());
    temperatureArray.mustDraw = false;
  }
}
