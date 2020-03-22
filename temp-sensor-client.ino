///////////////////////////////////////////////////
// TODO
// remove heavy string usage to save memory
// option class/functions that handle eeprom save/load
// add menu: button short press: next option; long press: change option
// - on/off auto load on start
// - on/off radio
// - on/off radio led and radio status
///////////////////////////////////////////////////

//////////////
// Includes //
//////////////

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NRFLite.h>
#include <U8g2lib.h>
#include <EEPROM.h>

///////////////////////////////////
// CONSTANTS                     //
// can be adjusted to your needs //
///////////////////////////////////

// define how often these Tasks are executed in ms
#define TASK_INTERVAL_BUTTON 10
#define TASK_INTERVAL_REQUESTTEMP 1000
#define TASK_INTERVAL_READTEMP 10
#define TASK_INTERVAL_RADIO 10000
#define TASK_INTERVAL_EEPROM 30000

// EEPROM addresses of some values
#define TASK_INTERVAL_REQUESTTEMP_EEPROM_ADDRESS 0
#define TEMPERATURE_POSITION_EEPROM_ADDRESS 1
// 128 values = 128 * 4 bytes (float) = 512 + 2
#define TEMPERATURE_GRAPH_EEPROM_ADDRESS 2

// Serial debugging settings
#define SERIAL_DEBUG true
#define SERIAL_SPEED 115200

// whether these functions should be logged, usually those run more frquent than the others
#define TASK_DEBUG_BUTTON false
#define TASK_DEBUG_READTEMP false

// serial names of tasks, I kept them 4 letters long to have a nice serial output
#define TASK_NAME_BUTTON "butn"
#define TASK_NAME_REQUESTTEMP "rqst"
#define TASK_NAME_READTEMP "read"
#define TASK_NAME_RADIO "rdio"
#define TASK_NAME_EEPROM "eprm"

// temperature sensor settings
#define TEMP_SENSOR_PIN 4
#define TEMP_SENSOR_STRING_DECIMALS 2

// button settings
#define BUTTON_PIN 7
#define BUTTON_LONG_PRESS_THRESHOLD 500
#define BUTTON_VERY_LONG_PRESS_THRESHOLD 2000

// rf2401l settings
#define RADIO_ENABLED true
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

#define LED_PIN 6

// these values can be selected with the one and only button
// it is the time each pixel represents the temperature
// totalDuration        128s, 256s, 640s,21:20m,   32m,42:40m,   64m, 2:08h,  4:16h,  6:24h, 10:40h,    12h, 21:20h,     1d,    1.5d,      2d,      3d,      4d,      5d,      7d,      8d,     10d,     12d,     16d,      24d
//singleDuration          1s,   2s,   5s,   10s,   15s,   20s,   30s,    1m,     2m,     3m,     5m,  5:38m,    10m, 11:15m, 16.875m,   22.5m,  33.75m,     45m,  56:15m,  78:45m,     90m,    112m,    135m,    180m,    4:30h
#define INTERVAL_VALUES 1000, 2000, 5000, 10000, 15000, 20000, 30000, 60000, 120000, 180000, 300000, 337500, 600000, 675000, 1012500, 1350000, 2025000, 2700000, 3375000, 4725000, 5400000, 6750000, 8100000, 10800000, 16200000
#define INTERVAL_VALUE_COUNT 25
// intervals greater that this will be executed every this value
#define INTERVAL_MAX_STEP_MILLIS 5000

// display settings

// SPI settings
#define DISPLAY_SPI true
#define DISPLAY_PIN_SPI_CHIP_SELECT_DS 10
#define DISPLAY_PIN_SPI_DATA_COMMAND 9
#define DISPLAY_PIN_SPI_RESET 8

// only HW constructors, SW constructors need different code later
#define DISPLAY_CONSTRUCTOR_SPI U8G2_SSD1306_128X64_VCOMH0_1_4W_HW_SPI
#define DISPLAY_CONSTRUCTOR_I2C U8G2_SSD1305_128X64_ADAFRUIT_1_HW_I2C
#define DISPLAY_ROTATION U8G2_R0
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_HEADER 16
#define DISPLAY_UPPER_LINE_OFFSET 6
#define DISPLAY_LOWER_LINE_OFFSET 14
#define DISPLAY_FONT u8g2_font_5x8_tf
//#define DISPLAY_FONT u8g2_font_5x7_tf
//#define DISPLAY_FONT u8g2_font_4x6_tf

// SPI constructor needs more parameters
// only HW constructors are considered here, SW needs different ones
#if DISPLAY_SPI
  DISPLAY_CONSTRUCTOR_SPI display(DISPLAY_ROTATION, DISPLAY_PIN_SPI_CHIP_SELECT_DS, DISPLAY_PIN_SPI_DATA_COMMAND, DISPLAY_PIN_SPI_RESET);
#else
  DISPLAY_CONSTRUCTOR_I2C display(DISPLAY_ROTATION);
#endif

///////////////////////////////////////////////////
// Task Class that allows pseudo parallelization //
// call .run() of all your Tasks in main()       //
///////////////////////////////////////////////////

class Task {
    uint32_t currentMillis;
    void (*functionPointer)();
    const char* functionName;

  public:
    uint32_t previousMillis;
    uint32_t interval;

    Task(void (*function)(), uint32_t new_interval, const char* name) {
      functionPointer = function;
      interval = new_interval;
      previousMillis = 0;
      functionName = name;
      return;
    }

    void run() {
      currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        #if SERIAL_DEBUG
        if (((strncmp(functionName, TASK_NAME_BUTTON, 4) != 0) || TASK_DEBUG_BUTTON) &&
        ((strncmp(functionName, TASK_NAME_READTEMP, 4) != 0) || TASK_DEBUG_READTEMP)){
          Serial.print(functionName);
          Serial.println(": " + String(currentMillis - previousMillis));
        }
        #endif
        previousMillis = currentMillis;
        functionPointer();
      }
    }
};

//////////////////////////////////////////
// ValueGraphArray stores measured data //
// and calculates optimal graph         //
//////////////////////////////////////////

bool mustDraw = true;

class ValueGraphArray {
  private:
    float deltaPerPixel;
    
  public:
    float minimum;
    float maximum;
    uint8_t currentPosition = 0;
    float values[DISPLAY_WIDTH] = {};
    
    uint32_t stepCount = 1;
    uint32_t currentStep = 0;
    float currentValue = 0;

  void add(float value){
    currentValue = currentValue * currentStep++ + value;
    currentValue /= currentStep;
    values[currentPosition] = currentValue;
    
    if (currentStep == stepCount){
      currentPosition = (currentPosition + 1) % DISPLAY_WIDTH;
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
      currentPosition = (currentPosition + 1) % DISPLAY_WIDTH;
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
}temperatureArray;

///////////////////////////////////
// Interval to String conversion //
///////////////////////////////////

#define SECOND 1000
#define MINUTE 60000
#define HOUR 3600000
#define DAY 86400000

String intervalToString(uint32_t interval){
  uint8_t value = 0;
  uint8_t decimal;
  char unit;

  if (interval < MINUTE){
    value = int(interval / SECOND);
    decimal = 0;
    unit = 's'; 
  }
  else if (interval < HOUR){
    value = int(interval / MINUTE);
    decimal = (interval - value * MINUTE) / SECOND;
    unit = 'm'; 
  }
  else if (interval < DAY){
    value = int(interval / HOUR);
    decimal = (interval - value * HOUR) / MINUTE;
    unit = 'h'; 
  }
  else{
    value = int(interval / DAY);
    decimal = (interval - value * DAY) / HOUR;
    unit = 'd'; 
  }

  String decimalWithLeadingZero = decimal < 10 ? "0" : "";
  return String(value) + ((decimal > 0) ? (":" + decimalWithLeadingZero + String(decimal)) : "") + unit;
}

/////////////////////////////////////////////////////////
// Request and Read Temperature Variables and Function //
/////////////////////////////////////////////////////////

OneWire oneWire(TEMP_SENSOR_PIN); 
DallasTemperature sensors(&oneWire);
float temperature;
int8_t templine;
bool tempIsRequested = false;
uint32_t tempReadTime;
// 12: highest, 0.0625 to 9: lowest, 0.5
uint8_t temperatureResolution = 12;

void requestTemperature(void){
  if (!tempIsRequested){
    sensors.requestTemperatures();
    tempReadTime = millis() + (750 / (1 << (12 - temperatureResolution)));
    tempIsRequested = true;
  }
}

Task taskRequestTemperature = Task(*requestTemperature, TASK_INTERVAL_REQUESTTEMP, TASK_NAME_REQUESTTEMP);

void readTemperature(void){
  if (tempIsRequested && (millis() >= tempReadTime)){
    temperature = sensors.getTempCByIndex(0);
    temperatureArray.add(temperature);
    #if SERIAL_DEBUG
    Serial.println("T " + String(temperature, 4));
    #endif
    tempIsRequested = false;
  }
}

Task taskReadTemperature = Task(*readTemperature, TASK_INTERVAL_READTEMP, TASK_NAME_READTEMP);

//////////////////////////////////////////
// Update EEPROM Variables and Function //
//////////////////////////////////////////

int8_t intervalPointer;
int8_t oldIntervalPointer;

void updateEeprom() {
  if (oldIntervalPointer != intervalPointer){
    #if SERIAL_DEBUG
    Serial.println("O " + String(oldIntervalPointer) + " -> N " + String(intervalPointer));
    #endif
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
    #if SERIAL_DEBUG
    Serial.println("I" + String(interval) + " IP" + String(intervalPointer));
    #endif
    temperatureArray.change_interval(interval > INTERVAL_MAX_STEP_MILLIS ? interval / INTERVAL_MAX_STEP_MILLIS : 1);
    taskRequestTemperature.interval = min(interval, INTERVAL_MAX_STEP_MILLIS);
    taskRequestTemperature.previousMillis = taskUpdateEeprom.previousMillis = millis();
}

///////////////////////////////////
// NRF2401L Struct and Variables //
// max 32 bytes RadioData        //
///////////////////////////////////

#if RADIO_ENABLED

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

#endif

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

///////////////////////////////////////////
// Send RadioData Variables and Function //
///////////////////////////////////////////

#if RADIO_ENABLED

void sendRadioData(){
    radioData.intervalReadTempGlobal = interval;
    radioData.temperature = temperature;
    radioData.intervalReadTempTask = taskRequestTemperature.interval;
    digitalWrite(LED_PIN, !radio.send(RADIO_ID_DESTINATION, &radioData, sizeof(radioData)) && RADIO_LED_ON_ERROR);
}

Task taskSendRadioData = Task(*sendRadioData, TASK_INTERVAL_RADIO, TASK_NAME_RADIO);

#endif

//////////////////////////////////
// Main Variables and Functions //
//////////////////////////////////

#if SERIAL_DEBUG
uint32_t drawTime;
uint32_t drawCount;
#endif

uint8_t headerState;
const uint8_t headerStateCounter = 5;

bool drawTemplines = true;
uint8_t templineDistance = 8;
const uint8_t templineDistanceMax = 16;

void setup() {
  #if SERIAL_DEBUG
  Serial.begin(SERIAL_SPEED);
  #endif
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  #if RADIO_ENABLED
  pinMode(LED_PIN, OUTPUT); 

  if (!radio.init(RADIO_ID_SOURCE, RADIO_PIN_CE, RADIO_PIN_CSN, RADIO_SPEED, RADIO_CHANNEL)){
    digitalWrite(LED_PIN, RADIO_LED_ON_ERROR);
    while (1);
  }  
  radioData.sourceRadioId = RADIO_ID_SOURCE;
  radioData.intervalRadio = TASK_INTERVAL_RADIO;
    #if SERIAL_DEBUG
    Serial.println("RDS " + String((int)sizeof(RadioData)));
    #endif
  #endif
  
  sensors.begin();
  sensors.setResolution(temperatureResolution);
  sensors.setWaitForConversion(false);
  
  display.begin();
  display.setFont(DISPLAY_FONT);
  display.setColorIndex(1);

  // load old data from EEPROM
  EEPROM.get(TASK_INTERVAL_REQUESTTEMP_EEPROM_ADDRESS, intervalPointer);
  oldIntervalPointer = intervalPointer %= INTERVAL_VALUE_COUNT;
  applyIntervalChanges();

  temperatureArray.readHistory();
  requestTemperature();
}

void loop() {
  // execute all Tasks, they check if it is their time
  #if RADIO_ENABLED
  taskSendRadioData.run();
  #endif
  taskRequestTemperature.run();
  taskReadTemperature.run();
  taskUpdateEeprom.run();
  taskReadButton.run();
  
  // cycle header options on shortPress
  if (buttonShortPress){
    buttonShortPress = false;
    mustDraw = true;
    headerState = headerState == headerStateCounter ? 0 : headerState + 1;
  }
  // change shown option
  if (buttonLongPress){
    buttonLongPress = false;
    mustDraw = true;
    switch(headerState) {
      case 0: intervalPointer = (intervalPointer + 1) % INTERVAL_VALUE_COUNT;
              applyIntervalChanges(); 
              break;
      case 1: drawTemplines = !drawTemplines;  
              break;
      case 2: templineDistance = templineDistance == templineDistanceMax ? 1 : templineDistance + 1;
              break;
      case 3: temperatureResolution = temperatureResolution == 12 ? 9 : temperatureResolution + 1;
              sensors.setResolution(temperatureResolution);
              break;
      case 4: temperatureArray.readHistory();  
              break;
      case 5: temperatureArray.writeHistory();
              break;
      default: 
              break;
    }
  }
  // longPress does the same or decreases a value
  if (buttonVeryLongPress){
    buttonVeryLongPress = false;
    mustDraw = true;
    switch(headerState) {
      case 0: intervalPointer = intervalPointer == 0 ? INTERVAL_VALUE_COUNT - 1 : intervalPointer - 1;
              applyIntervalChanges(); 
              break;
      case 1: drawTemplines = !drawTemplines;  
              break;
      case 2: templineDistance = templineDistance == 1 ? templineDistanceMax : templineDistance - 1;
              break;
      case 3: temperatureResolution = temperatureResolution == 9 ? 12 : temperatureResolution - 1;
              sensors.setResolution(temperatureResolution);
              break;
      case 4: temperatureArray.readHistory();  
              break;
      case 5: temperatureArray.writeHistory();
              break;
      default: 
              break;
    }
  }  

  // various actions trigger mustDraw, e.g. new temperature or changed option
  if (mustDraw){
    #if SERIAL_DEBUG
    drawTime = millis();
    drawCount = 0;
    #endif
    // partly buffered displays need this kind of loop
    display.firstPage();
    do {
      #if SERIAL_DEBUG
      drawCount++;
      #endif
      
      // draw header information
      switch(headerState) {
        case 0: {
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
                  break;
                }
        case 1: display.setCursor(0, DISPLAY_UPPER_LINE_OFFSET);
                display.print(F("Draw Temperature Lines")); 
                display.setCursor(0, DISPLAY_LOWER_LINE_OFFSET);
                display.print(drawTemplines);   
                break;
        case 2: display.setCursor(0, DISPLAY_UPPER_LINE_OFFSET);
                display.print(F("Temperature Line Distance")); 
                display.setCursor(0, DISPLAY_LOWER_LINE_OFFSET);
                display.print(templineDistance);   
                break;
        case 3: display.setCursor(0, DISPLAY_UPPER_LINE_OFFSET);
                display.print(F("Temperature Resolution")); 
                display.setCursor(0, DISPLAY_LOWER_LINE_OFFSET);
                display.print(String(0.0625 * (1 << (12 - temperatureResolution)), 4));
                display.setCursor(35, DISPLAY_LOWER_LINE_OFFSET);
                display.print(temperatureResolution);
                display.setCursor(50, DISPLAY_LOWER_LINE_OFFSET);
                display.print(750 / (1 << (12 - temperatureResolution)));
                break;
        case 4: display.setCursor(0, DISPLAY_UPPER_LINE_OFFSET);
                display.print(F("READ Graph History"));
                break;
        case 5: display.setCursor(0, DISPLAY_UPPER_LINE_OFFSET);
                display.print(F("WRITE Graph History"));  
                break;
        default: 
                break;
      }

      // draw temperature graph
      for (uint8_t i=0; i < DISPLAY_WIDTH; i++){
        uint8_t y_position = temperatureArray.y_from_position((DISPLAY_WIDTH - i + temperatureArray.currentPosition) % DISPLAY_WIDTH);
        display.drawPixel(DISPLAY_WIDTH - 1 - i, y_position);
      }

      // draw lines if enabled in options
      if (drawTemplines){
        templine = ceil(temperatureArray.minimum);
        while (templine <= temperatureArray.maximum){
          for (uint8_t x = 0; x < DISPLAY_WIDTH; x += templineDistance){
            display.drawPixel(x, temperatureArray.y_from_temperature(templine));
          }
          templine++;
        }
      }

    } while (display.nextPage());
    mustDraw = false;
    #if SERIAL_DEBUG
    Serial.println("draw: " + String(millis() - drawTime) + " " + String(drawCount));
    #endif
  }
}
