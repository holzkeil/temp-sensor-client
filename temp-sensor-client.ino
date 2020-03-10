///////////////////////////////////////////////////
// TODO
// compiler define selects I2C or SPI
// compiler define turns NRF on/off
// compiler define turns serial debugging on/off
// remove heavy string usage to save memory
///////////////////////////////////////////////////

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

#define TASK_INTERVAL_BUTTON 10
#define TASK_INTERVAL_REQUESTTEMP 1000
#define TASK_INTERVAL_READTEMP 10
#define TASK_INTERVAL_RADIO 10000
#define TASK_INTERVAL_EEPROM 30000
#define TASK_INTERVAL_REQUESTTEMP_EEPROM_ADDRESS 0

#define TEMPERATURE_POSITION_EEPROM_ADDRESS 1
#define TEMPERATURE_GRAPH_EEPROM_ADDRESS 2

#define TASK_NAME_BUTTON "butn"
#define TASK_NAME_REQUESTTEMP "rqst"
#define TASK_NAME_READTEMP "read"
#define TASK_NAME_RADIO "rdio"
#define TASK_NAME_EEPROM "eprm"

#define TEMP_SENSOR_PIN 4
#define TEMP_SENSOR_RESOLUTION 12  // 12: highest, 0.0625 to 9: lowest, 0.5
#define TEMP_SENSOR_STRING_DECIMALS 2

#define BUTTON_PIN 7
#define BUTTON_LONG_PRESS_THRESHOLD 2000
#define BUTTON_VERY_LONG_PRESS_THRESHOLD 5000

#define LED_PIN 6

#define RADIO_ID_SOURCE 1
#define RADIO_ID_DESTINATION 0
#define RADIO_PIN_CE 2
#define RADIO_PIN_CSN 3
// By default, 'init' configures the radio to use a 2MBPS bitrate on channel 100 (channels 0-125 are valid).
// Both the RX and TX radios must have the same bitrate and channel to communicate with each other.
//#define RADIO_SPEED NRFLite::BITRATE250KBPS
//#define RADIO_SPEED NRFLite::BITRATE1MBPS
#define RADIO_SPEED NRFLite::BITRATE2MBPS
#define RADIO_CHANNEL 100 // 0 - 125
#define RADIO_LED_ON_ERROR true
 
// these values can be selected with the one and only button
// it is the time each pixel represents the temperature
// totalDuration        128s, 256s, 640s,21:20m,   32m,42:40m,   64m, 2:08h,  4:16h,  6:24h, 10:40h,    12h, 21:20h,     1d,    1.5d,      2d,      3d,      5d,      7d,     10d,     14d,      21d
//singleDuration          1s,   2s,   5s,   10s,   15s,   20s,   30s,    1m,     2m,     3m,     5m,  5:38m,    10m, 11:15m, 16.875m,   22.5m,  33.75m,  56:15m,  78.75m,    112m,  157.5m,    3:56h
#define INTERVAL_VALUES 1000, 2000, 5000, 10000, 15000, 20000, 30000, 60000, 120000, 180000, 300000, 337500, 600000, 675000, 1012500, 1350000, 2025000, 3375000, 4725000, 6750000, 9450000, 14175000
#define INTERVAL_VALUE_COUNT 22
// intervals greater that this will be executed every this value
#define INTERVAL_MAX_STEP_MILLIS 5000

#define DISPLAY_PIN_CHIP_SELECT_DS 10
#define DISPLAY_PIN_DATA_COMMAND 9
#define DISPLAY_PIN_RESET 8

#define DISPLAY_CONSTRUCTOR U8G2_SSD1306_128X64_VCOMH0_1_4W_HW_SPI
#define DISPLAY_ROTATION U8G2_R0
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_HEADER 16
#define DISPLAY_UPPER_LINE_OFFSET 6
#define DISPLAY_LOWER_LINE_OFFSET 14
#define DISPLAY_FONT u8g2_font_5x8_tf
//#define DISPLAY_FONT u8g2_font_5x7_tf
//#define DISPLAY_FONT u8g2_font_4x6_tf

#define SERIAL_SPEED 115200

#define TASK_DEBUG_BUTTON false
#define TASK_DEBUG_READTEMP false

///////////////////////////////////////////////////
// Task Class that allows pseudo parallelization //
// call .run() of all your Tasks in main()       //
///////////////////////////////////////////////////

class Task {
    uint32_t currentMillis;
    void (*functionPointer)();
    String functionName;

  public:
    uint32_t previousMillis;
    uint32_t interval;

    Task(void (*function)(), uint32_t new_interval, String name) {
      functionPointer = function;
      interval = new_interval;
      previousMillis = 0;
      functionName = name;
      return;
    }

    run() {
      currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        if ((functionName != TASK_NAME_BUTTON || TASK_DEBUG_BUTTON) && (functionName != TASK_NAME_READTEMP || TASK_DEBUG_READTEMP)){
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
    mustDraw = true;
  }
  
  uint8_t y_from_position(uint8_t position){
    return y_from_temperature(values[position]);
  }

  uint8_t y_from_temperature(float temperature){
    return DISPLAY_HEIGHT - max(1, (temperature - minimum) / deltaPerPixel);
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

  void readHistory(){
    EEPROM.get(TEMPERATURE_POSITION_EEPROM_ADDRESS, currentPosition);
    for (uint8_t i=0; i<DISPLAY_WIDTH; i++){
      EEPROM.get(TEMPERATURE_GRAPH_EEPROM_ADDRESS + i * sizeof(float), values[i]);
    }
  }
  
  void writeHistory(){
    EEPROM.put(TEMPERATURE_POSITION_EEPROM_ADDRESS, currentPosition);
    for (uint8_t i=0; i<DISPLAY_WIDTH; i++){
      EEPROM.put(TEMPERATURE_GRAPH_EEPROM_ADDRESS + i * sizeof(float), values[i]);
    }
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

/////////////////////////////////////////////////////////
// Request and Read Temperature Variables and Function //
/////////////////////////////////////////////////////////

OneWire oneWire(TEMP_SENSOR_PIN); 
DallasTemperature sensors(&oneWire);
ValueGraphArray temperatureArray = ValueGraphArray();
float temperature;
int8_t templine;
bool tempIsRequested = false;
uint32_t tempReadTime;

void requestTemperature(void){
  if (!tempIsRequested){
    sensors.requestTemperatures();
    tempReadTime = millis() + (750 / (1 << (12 - TEMP_SENSOR_RESOLUTION)));
    tempIsRequested = true;
  }
}

Task taskRequestTemperature = Task(*requestTemperature, TASK_INTERVAL_REQUESTTEMP, TASK_NAME_REQUESTTEMP);

void readTemperature(void){
  if (tempIsRequested && (millis() >= tempReadTime)){
    temperature = sensors.getTempCByIndex(0);
    temperatureArray.add(temperature);
    Serial.println("T " + String(temperature, 4));
    tempIsRequested = false;
  }
}

Task taskReadTemperature = Task(*readTemperature, TASK_INTERVAL_READTEMP, TASK_NAME_READTEMP);

//////////////////////////////////////////////
// Read Button Press Variables and Function //
//////////////////////////////////////////////

bool buttonNewState = true;
bool buttonOldState = true;
bool buttonShortPress = false;
bool buttonLongPress = false;
bool buttonVeryLongPress = false;
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
      if (buttonCurrentMillis - buttonTimer >= BUTTON_VERY_LONG_PRESS_THRESHOLD) {
        buttonVeryLongPress = true;
      }
      else if (buttonCurrentMillis - buttonTimer >= BUTTON_LONG_PRESS_THRESHOLD) {
        buttonLongPress = true;
      }
      else {
        buttonShortPress = true;
      }
    }
  }
  buttonOldState = buttonNewState;
}

Task taskReadButton = Task(*readButton, TASK_INTERVAL_BUTTON, TASK_NAME_BUTTON);

//////////////////////////////////////////
// Update EEPROM Variables and Function //
//////////////////////////////////////////

int8_t intervalPointer;
int8_t oldIntervalPointer;

void updateEeprom() {
  if (oldIntervalPointer != intervalPointer){
    Serial.println("O " + String(oldIntervalPointer) + " -> N " + String(intervalPointer));
    EEPROM.update(TASK_INTERVAL_REQUESTTEMP_EEPROM_ADDRESS, oldIntervalPointer = intervalPointer);
  }
}

Task taskUpdateEeprom = Task(*updateEeprom, TASK_INTERVAL_EEPROM, TASK_NAME_EEPROM);

////////////////////////////////////////////
// Change Interval Variables and Function //
////////////////////////////////////////////

const uint32_t intervalValues[INTERVAL_VALUE_COUNT] = {INTERVAL_VALUES};
uint32_t interval;

void applyIntervalChanges(){
    interval = intervalValues[intervalPointer];
    Serial.println("I" + String(interval) + " IP" + String(intervalPointer));
    temperatureArray.change_interval(interval > INTERVAL_MAX_STEP_MILLIS ? interval / INTERVAL_MAX_STEP_MILLIS : 1);
    taskRequestTemperature.interval = min(interval, INTERVAL_MAX_STEP_MILLIS);
    taskRequestTemperature.previousMillis = taskUpdateEeprom.previousMillis = millis();
}

///////////////////////////////////////////
// Send RadioData Variables and Function //
///////////////////////////////////////////

void sendRadioData(){
    radioData.intervalReadTempGlobal = interval;
    radioData.temperature = temperature;
    radioData.intervalReadTempTask = taskRequestTemperature.interval;
    digitalWrite(LED_PIN, !radio.send(RADIO_ID_DESTINATION, &radioData, sizeof(radioData)) && RADIO_LED_ON_ERROR);
}

Task taskSendRadioData = Task(*sendRadioData, TASK_INTERVAL_RADIO, TASK_NAME_RADIO);

//////////////////////////////////
// Main Variables and Functions //
//////////////////////////////////

#define SECOND 1000
#define MINUTE 60000
#define HOUR 3600000
#define DAY 86400000

String intervalToString(uint32_t interval){
  uint8_t value = 0;
  uint8_t decimal;
  String unit;

  if (interval < MINUTE){
    value = int(interval / SECOND);
    decimal = 0;
    unit = F("s"); 
  }
  else if (interval < HOUR){
    value = int(interval / MINUTE);
    decimal = (interval - value * MINUTE) / SECOND;
    unit = F("m"); 
  }
  else if (interval < DAY){
    value = int(interval / HOUR);
    decimal = (interval - value * HOUR) / MINUTE;
    unit = F("h"); 
  }
  else{
    value = int(interval / DAY);
    decimal = (interval - value * DAY) / HOUR;
    unit = F("d"); 
  }
  
  String decimalWithLeadingZero = decimal < 10 ? String("0") : String("");
  return String(value) + ((decimal > 0) ? (":" + decimalWithLeadingZero + String(decimal)) : "") + unit;
}

DISPLAY_CONSTRUCTOR display(DISPLAY_ROTATION, DISPLAY_PIN_CHIP_SELECT_DS, DISPLAY_PIN_DATA_COMMAND, DISPLAY_PIN_RESET);
uint32_t drawTime;
uint32_t drawCount;

void setup() {
  Serial.begin(SERIAL_SPEED);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT); 

  if (!radio.init(RADIO_ID_SOURCE, RADIO_PIN_CE, RADIO_PIN_CSN, RADIO_SPEED, RADIO_CHANNEL)){
    digitalWrite(LED_PIN, RADIO_LED_ON_ERROR);
    while (1);
  }  
  radioData.sourceRadioId = RADIO_ID_SOURCE;
  radioData.intervalRadio = TASK_INTERVAL_RADIO;

  Serial.println("RDS " + String(sizeof(RadioData)));
  
  sensors.begin();
  sensors.setResolution(TEMP_SENSOR_RESOLUTION);
  sensors.setWaitForConversion(false);
  
  display.begin();
  display.setFont(DISPLAY_FONT);
  display.setColorIndex(1);

  EEPROM.get(TASK_INTERVAL_REQUESTTEMP_EEPROM_ADDRESS, intervalPointer);
  oldIntervalPointer = intervalPointer %= INTERVAL_VALUE_COUNT;
  applyIntervalChanges();

  temperatureArray.readHistory();
  requestTemperature();
}

void loop() {
  taskSendRadioData.run();
  taskRequestTemperature.run();
  taskReadTemperature.run();
  taskUpdateEeprom.run();
  taskReadButton.run();
  
  if (buttonShortPress){
    intervalPointer = ++intervalPointer % INTERVAL_VALUE_COUNT;
    applyIntervalChanges();
    buttonShortPress = false;
  }
  if (buttonLongPress){
    temperatureArray.readHistory();
    buttonLongPress = false;
  }
  if (buttonVeryLongPress){
    temperatureArray.writeHistory();
    buttonVeryLongPress = false;
  }  

  if (temperatureArray.mustDraw){
    drawTime = millis();
    drawCount = 0;
    display.firstPage();
    do {
      drawCount++;
      display.setCursor(0, DISPLAY_UPPER_LINE_OFFSET);
      String upperRow = "H " + String(temperatureArray.maximum, TEMP_SENSOR_STRING_DECIMALS);
      if (temperatureArray.stepCount > 1){
        upperRow += " " + String(temperatureArray.values[(temperatureArray.currentPosition + DISPLAY_WIDTH - 1) % DISPLAY_WIDTH], TEMP_SENSOR_STRING_DECIMALS); 
        upperRow += " " + String(temperatureArray.currentValue, TEMP_SENSOR_STRING_DECIMALS);
        upperRow += " " + String(temperature, TEMP_SENSOR_STRING_DECIMALS);
      }
      else{
        upperRow += " " + String(temperatureArray.values[(temperatureArray.currentPosition + DISPLAY_WIDTH - 3) % DISPLAY_WIDTH], TEMP_SENSOR_STRING_DECIMALS); 
        upperRow += " " + String(temperatureArray.values[(temperatureArray.currentPosition + DISPLAY_WIDTH - 2) % DISPLAY_WIDTH], TEMP_SENSOR_STRING_DECIMALS); 
        upperRow += " " + String(temperatureArray.values[(temperatureArray.currentPosition + DISPLAY_WIDTH - 1) % DISPLAY_WIDTH], TEMP_SENSOR_STRING_DECIMALS); 
      }
      display.print(upperRow); 
      
      display.setCursor(0, DISPLAY_LOWER_LINE_OFFSET);
      String lowerRow = "L " + String(temperatureArray.minimum, TEMP_SENSOR_STRING_DECIMALS);
      if (temperatureArray.stepCount > 1){
        lowerRow += " " + String(temperatureArray.currentStep) + "/" + String(temperatureArray.stepCount); 
      }
      lowerRow += " " + intervalToString(interval) + " " + intervalToString(interval * DISPLAY_WIDTH);
      display.print(lowerRow);
      
      for (uint8_t i=0; i < DISPLAY_WIDTH; i++){
        uint8_t y_position = temperatureArray.y_from_position((DISPLAY_WIDTH - i + temperatureArray.currentPosition) % DISPLAY_WIDTH);
        display.drawPixel(DISPLAY_WIDTH - 1 - i, y_position);
      }

      templine = ceil(temperatureArray.minimum);
      while (templine <= temperatureArray.maximum){
        for (uint8_t x = 0; x < DISPLAY_WIDTH; x+=4){
          display.drawPixel(x, temperatureArray.y_from_temperature(templine));
        }
        templine++;
      }

    } while (display.nextPage());
    temperatureArray.mustDraw = false;
    Serial.println("draw: " + String(millis() - drawTime) + " " + String(drawCount));
  }
}
