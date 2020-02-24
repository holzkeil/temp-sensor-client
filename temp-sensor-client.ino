#include <OneWire.h>
#include <DallasTemperature.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <NRFLite.h>
#include <EEPROM.h>

// TODO
// -1 value bug, not visible
// send struct includes: interval
// sub interval measuring
// display show last avarage, interval counter, and current sub interval

#define PIN_TEMPSENSOR 4
#define TEMPRESOLUTION 12  // resolution 12 (highest) to 9 (lowest)
#define FLOAT_LENGTH 1
#define PIN_BUTTON 7
#define PIN_LED 6

#define SOURCE_RADIO_ID 1
#define DESTINATION_RADIO_ID 0
#define PIN_RADIO_CE 2
#define PIN_RADIO_CSN 3

#define INTERVAL_VALUE_COUNT 16
#define INTERVAL_EEPROM_ADDRESS 0

#define PIN_OLED_CHIP_SELECT_DS 10
#define PIN_OLED_DATA_COMMAND 9
#define PIN_OLED_RESET 8

#define DISPLAY_HEIGHT 64
#define DISPLAY_HEADER 16
#define DISPLAY_WIDTH 128

OneWire oneWire(PIN_TEMPSENSOR); 
DallasTemperature sensors(&oneWire);
U8G2_SSD1306_128X64_VCOMH0_1_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CHIP_SELECT_DS, PIN_OLED_DATA_COMMAND, PIN_OLED_RESET);

// max 32 bytes
struct RadioPacket
{
    uint8_t FromRadioId;
    uint32_t OnTimeMillis;
};

NRFLite _radio;
RadioPacket _radioData;

class Task {
    unsigned long previous_millis;
    unsigned long current_millis;
    unsigned long interval_values[INTERVAL_VALUE_COUNT] = {1000, 2000, 5000, 10000, 15000, 30000, 60000, 120000, 300000, 600000, 675000, 1012500, 1350000, 2025000, 4725000, 9450000};
    int8_t interval_pointer = 0;
    void (*fun)();
    String func_name;

  public:
    unsigned long interval;

    Task(void (*func)(), int ival, String name) {
      fun = func;
      if (String("temp") == name){
        EEPROM.get(INTERVAL_EEPROM_ADDRESS, interval_pointer);
        interval_pointer--;     
        change_interval();
      }
      else{
        interval = ival;
      }
      previous_millis = 0;
      func_name = name;
      return;
    }

    void change_interval(){
      interval_pointer = ++interval_pointer % INTERVAL_VALUE_COUNT;
      interval = interval_values[interval_pointer];
      Serial.println("Interval: " + String(interval) + "IntervalPointer: " + String(interval_pointer));
      EEPROM.update(INTERVAL_EEPROM_ADDRESS, interval_pointer);
    }
    
    run() {
      current_millis = millis();
      if (current_millis - previous_millis >= interval) {
        Serial.println("Running function '" + func_name + "' after: " + String(current_millis - previous_millis));
        previous_millis = current_millis;
        fun();
      }
    }
};

class TemperatureArray {
   public:
     uint8_t current = 0;
     float minimum;
     float maximum;
     float values[DISPLAY_WIDTH] = {};
     bool init_done = false;
     bool isDrawn = false;
     float delta;
     float delta_per_pixel;
    
    TemperatureArray(){
      return;
    }

  void add(float temp){
    if (!init_done){
      for (int i=0; i < DISPLAY_WIDTH; i++){
        values[i] = temp;
      }
      init_done = true;
    }
    values[current] = temp;
    current = ++current % DISPLAY_WIDTH;
    minimum = values[0];
    maximum = values[0];
    for (uint8_t i=1;i<DISPLAY_WIDTH;i++){
      if (values[i] < minimum) minimum = values[i];
      if (values[i] > maximum) maximum = values[i];
    }
    delta = maximum - minimum;
    if (delta != 0.0){ 
      delta_per_pixel = delta / (DISPLAY_HEIGHT - DISPLAY_HEADER);
    }
    else {
      delta_per_pixel = 1.0;
    }
    isDrawn = false;
  }

  unsigned int get_val(uint8_t position){
    float t;
    if (values[position] ==  minimum){
      t = 1;
    }
    else{
      t = (values[position] - minimum) / delta_per_pixel;
    }
    return DISPLAY_HEIGHT - ceil(t);
  }

};

TemperatureArray temperatureArray = TemperatureArray();

void readTemperature(void){
  sensors.requestTemperatures(); 
  temperatureArray.add(sensors.getTempCByIndex(0));

  _radioData.OnTimeMillis = (int)(sensors.getTempCByIndex(0)*1000);
  digitalWrite(PIN_LED, !_radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData)));
}

Task ttemp = Task(*readTemperature, 1000, "temp");

int old_state = true;
bool short_press = false;
bool long_press = false;
unsigned int long_press_threshold = 1500;
unsigned long button_timer = 0;

void readButton() {
  int new_state = digitalRead(PIN_BUTTON);
  if (new_state != old_state) {
    if (new_state == LOW) {
      button_timer = millis();
    }
    else {
      unsigned long current_millis = millis();
      if (current_millis - button_timer >= long_press_threshold) {
        long_press = true;
      }
      else {
        short_press = true;
      }
    }
  }
  old_state = new_state;
}

Task tbutton = Task(*readButton, 50, "button");

void setup() {
  Serial.begin(115200);
  
  if (!_radio.init(SOURCE_RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN)){
    Serial.println("Cannot communicate with radio");
    while (1);
  }  
  _radioData.FromRadioId = SOURCE_RADIO_ID;
  
  sensors.begin();
  sensors.setResolution(TEMPRESOLUTION);
  
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.setColorIndex(1);
  
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  readTemperature();
}

float duration = 0;
String text;

void loop() {
  ttemp.run();
  tbutton.run();
  if (short_press || long_press){
    Serial.println("Long press");
    ttemp.change_interval();
    temperatureArray.isDrawn = false;
    short_press = false;
    long_press = false;
  }

  if (!(temperatureArray.isDrawn)){
    u8g2.firstPage();
    do {
      u8g2.setCursor(0, 6);
      u8g2.print("H " + String(temperatureArray.maximum, FLOAT_LENGTH)); 
      u8g2.setCursor(0, 14);
      u8g2.print("L " + String(temperatureArray.minimum, FLOAT_LENGTH)); 
      u8g2.setCursor(100, 6);
      u8g2.print(String(temperatureArray.values[(temperatureArray.current + 127)%DISPLAY_WIDTH], FLOAT_LENGTH)); 
      if (ttemp.interval < 60000){
        duration = ttemp.interval / 1000.0;
        text = "s"; 
      }
      else if (ttemp.interval < 3600000){
        duration = ttemp.interval / 60000.0;
        text = "m"; 
      }
      else {
        duration = ttemp.interval / 3600000.0;
        text = "h"; 
      }
      u8g2.setCursor(54, 14);
      u8g2.print(String(duration) + text + " " + String((int)(duration*DISPLAY_WIDTH)) + text);
      for (int i=0; i<DISPLAY_WIDTH;i++){
        unsigned int val = temperatureArray.get_val((temperatureArray.current - i + 127)%DISPLAY_WIDTH);
        u8g2.drawPixel(127 - i, val);
      }
    } while ( u8g2.nextPage() );
    temperatureArray.isDrawn = true;
  }
}
