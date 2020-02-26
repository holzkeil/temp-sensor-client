#include <OneWire.h>
#include <DallasTemperature.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <NRFLite.h>
#include <EEPROM.h>

// TODO
// rename simpletask to task, derive interval changeable task from (simple) task
// sub interval measuring
// display show last avarage, interval counter, and current sub interval

#define PIN_TEMPSENSOR 4
#define TEMPRESOLUTION 12  // resolution 12 (highest) to 9 (lowest)
#define FLOAT_LENGTH 1

#define PIN_BUTTON 7
#define LONG_PRESS_THRESHOLD 1500

#define PIN_LED 6

#define SOURCE_RADIO_ID 1
#define DESTINATION_RADIO_ID 0
#define PIN_RADIO_CE 2
#define PIN_RADIO_CSN 3

#define INTERVAL_VALUES 1000, 2000, 5000, 10000, 15000, 30000, 60000, 120000, 300000, 600000, 675000, 1012500, 1350000, 2025000, 4725000, 9450000
#define INTERVAL_VALUE_COUNT 16
#define INTERVAL_EEPROM_ADDRESS 0

#define PIN_OLED_CHIP_SELECT_DS 10
#define PIN_OLED_DATA_COMMAND 9
#define PIN_OLED_RESET 8

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_HEADER 16
#define DISPLAY_FONT u8g2_font_5x8_tf

#define SERIAL_SPEED 115200

OneWire oneWire(PIN_TEMPSENSOR); 
DallasTemperature sensors(&oneWire);
U8G2_SSD1306_128X64_VCOMH0_1_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CHIP_SELECT_DS, PIN_OLED_DATA_COMMAND, PIN_OLED_RESET);

// max 32 bytes
struct RadioData
{
    uint8_t sourceRadioId;
    float temperature;
    uint32_t interval;
};

NRFLite radio;
RadioData radioData;

class Task {
    uint32_t previousMillis;
    uint32_t currentMillis;
    uint32_t intervalValues[INTERVAL_VALUE_COUNT] = {INTERVAL_VALUES};
    void (*functionPointer)();
    String functionName;

  public:
    uint32_t interval;
    int8_t intervalPointer = 0;

    Task(void (*function)(), uint16_t eepromAddress, String name) {
      functionPointer = function;
      EEPROM.get(INTERVAL_EEPROM_ADDRESS, intervalPointer);
      intervalPointer %= INTERVAL_VALUE_COUNT;
      set_interval();
      previousMillis = 0;
      functionName = name;
      return;
    }

    void change_interval_pointer(){
      intervalPointer = ++intervalPointer % INTERVAL_VALUE_COUNT;
      set_interval();
      }

    void set_interval(){
      interval = intervalValues[intervalPointer];
      radioData.interval = interval;
      Serial.println("I" + String(interval) + " IP" + String(intervalPointer));
    }
    
    run() {
      currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        Serial.println(functionName + ": " + String(currentMillis - previousMillis));
        previousMillis = currentMillis;
        functionPointer();
      }
    }
};

class SimpleTask {
    uint32_t previousMillis;
    uint32_t currentMillis;
    void (*functionPointer)();
    String functionName;
    uint32_t interval;

   public:
    SimpleTask(void (*function)(), uint32_t ival, String name) {
      functionPointer = function;
      interval = ival;
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
        Serial.println(functionName + ": " + String(currentMillis - previousMillis));
        previousMillis = currentMillis;
        functionPointer();
      }
    }
};

class TemperatureArray {
   public:
     uint8_t current = 0;
     float minimum;
     float maximum;
     float values[DISPLAY_WIDTH] = {};
     bool initDone = false;
     bool mustDraw = true;
     float deltaPerPixel;
    
    TemperatureArray(){
      return;
    }

  void add(float temp){
    if (!initDone){
      for (uint8_t i=0; i < DISPLAY_WIDTH; i++){
        values[i] = temp;
      }
      initDone = true;
    }
    values[current] = temp;
    current = ++current % DISPLAY_WIDTH;
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

  uint8_t get_val(uint8_t position){
    return DISPLAY_HEIGHT - max(1, (values[position] - minimum) / deltaPerPixel);
  }

};

TemperatureArray temperatureArray = TemperatureArray();

void readTemperature(void){
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);
  temperatureArray.add(temperature);
  radioData.temperature = temperature;
  Serial.println("T " + String(temperature, 4));
  digitalWrite(PIN_LED, !radio.send(DESTINATION_RADIO_ID, &radioData, sizeof(radioData)));
}

Task taskReadTemperature = Task(*readTemperature, INTERVAL_EEPROM_ADDRESS, "temp");

bool buttonOldState = true;
bool buttonShortPress = false;
bool buttonLongPress = false;
uint32_t buttonTimer = 0;

void readButton() {
  bool buttonNewState = digitalRead(PIN_BUTTON);
  if (buttonNewState != buttonOldState) {
    if (buttonNewState == LOW) {
      buttonTimer = millis();
    }
    else {
      uint32_t buttonCurrentMillis = millis();
      if (buttonCurrentMillis - buttonTimer >= LONG_PRESS_THRESHOLD) {
        buttonLongPress = true;
      }
      else {
        buttonShortPress = true;
      }
    }
  }
  buttonOldState = buttonNewState;
}

SimpleTask taskReadButton = SimpleTask(*readButton, 50, "button");

uint32_t oldIntervalPointer = taskReadTemperature.intervalPointer;

void updateEeprom() {
  if (oldIntervalPointer != taskReadTemperature.intervalPointer){
    Serial.println("O " + String(oldIntervalPointer) + " -> N " + String(taskReadTemperature.intervalPointer));
    EEPROM.update(INTERVAL_EEPROM_ADDRESS, oldIntervalPointer = taskReadTemperature.intervalPointer);
  }
}

SimpleTask taskUpdateEeprom = SimpleTask(*updateEeprom, 30000, "eeprom");

void setup() {
  Serial.begin(SERIAL_SPEED);
  
  if (!radio.init(SOURCE_RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN)){
    Serial.println("Radio Error");
    while (1);
  }  
  radioData.sourceRadioId = SOURCE_RADIO_ID;
  
  sensors.begin();
  sensors.setResolution(TEMPRESOLUTION);
  
  u8g2.begin();
  u8g2.setFont(DISPLAY_FONT);
  u8g2.setColorIndex(1);
  
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  readTemperature();
}

float duration = 0;
String text;

void loop() {
  taskReadTemperature.run();
  taskUpdateEeprom.run();
  taskReadButton.run();
  if (buttonShortPress || buttonLongPress){
    taskReadTemperature.change_interval_pointer();
    taskUpdateEeprom.reset();
    temperatureArray.mustDraw = true;
    buttonShortPress = false;
    buttonLongPress = false;
  }

  if (temperatureArray.mustDraw){
    u8g2.firstPage();
    do {
      u8g2.setCursor(0, 6);
      u8g2.print("H " + String(temperatureArray.maximum, FLOAT_LENGTH)); 
      u8g2.setCursor(0, 14);
      u8g2.print("L " + String(temperatureArray.minimum, FLOAT_LENGTH)); 
      u8g2.setCursor(100, 6);
      u8g2.print(String(temperatureArray.values[(temperatureArray.current + 127)%DISPLAY_WIDTH], FLOAT_LENGTH)); 
      if (taskReadTemperature.interval < 60000){
        duration = taskReadTemperature.interval / 1000.0;
        text = "s"; 
      }
      else if (taskReadTemperature.interval < 3600000){
        duration = taskReadTemperature.interval / 60000.0;
        text = "m"; 
      }
      else {
        duration = taskReadTemperature.interval / 3600000.0;
        text = "h"; 
      }
      u8g2.setCursor(54, 14);
      u8g2.print(String(duration) + text + " " + String((int)(duration*DISPLAY_WIDTH)) + text);
      for (uint8_t i=0; i < DISPLAY_WIDTH; i++){
        uint8_t val = temperatureArray.get_val((DISPLAY_WIDTH - 1 - i + temperatureArray.current)%DISPLAY_WIDTH);
        u8g2.drawPixel(DISPLAY_WIDTH - 1 - i, val);
      }
    } while ( u8g2.nextPage() );
    temperatureArray.mustDraw = false;
  }
}
