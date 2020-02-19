#include <OneWire.h>
#include <DallasTemperature.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <NRFLite.h>

#define TEMPSENSORPIN 4
#define TEMPRESOLUTION 12
#define BUTTONPIN 7
#define LEDPIN 6

#define RADIO_ID 1             // Our radio's id.
#define DESTINATION_RADIO_ID 0 // Id of the radio we will transmit to.
#define PIN_RADIO_CE 2
#define PIN_RADIO_CSN 3

OneWire oneWire(TEMPSENSORPIN); 
DallasTemperature sensors(&oneWire);

#define OLED_CS 10 // Chip Select, DS
#define OLED_DC 9 // Data/Command
#define OLED_RESET 8

U8G2_SSD1306_128X64_VCOMH0_1_4W_HW_SPI u8g2(U8G2_R0, OLED_CS, OLED_DC, OLED_RESET);

struct RadioPacket // Any packet up to 32 bytes can be sent.
{
    uint8_t FromRadioId;
    uint32_t OnTimeMillis;
    uint32_t FailedTxCount;
};

NRFLite _radio;
RadioPacket _radioData;

class Task {
    unsigned long previous_millis;
    unsigned long current_millis;
    unsigned long interval_values[16] = {1000, 2000, 5000, 10000, 15000, 30000, 60000, 120000, 300000, 600000, 675000, 1012500, 1350000, 2025000, 4725000, 9450000};
    int interval_pointer = 0;
    void (*fun)();
    String func_name;

  public:
    unsigned long interval;

    Task(void (*func)(), int ival, String name) {
      fun = func;
      interval = ival;
      previous_millis = 0;
      func_name = name;
      return;
    }

    void change_interval(void){
      interval_pointer = ++interval_pointer % 16;
      interval = interval_values[interval_pointer];
      Serial.println("New interval: " + String(interval));
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
     unsigned int current = 0;
     float minimum = 10000;
     float maximum= -10000;
     const static unsigned int count = 128;
     float values[count] = {};
     bool init_done = false;
     bool isDrawn = false;
     unsigned int display_height;
    
    TemperatureArray(unsigned int height){
      display_height = height;
      return;
    }

  void add(float temp){
    values[current] = temp;
    current = ++current % count;
    float temp_min = values[0];
    float temp_max = values[0];
    for (int i=0;i<128;i++){
      if (values[i] < temp_min) temp_min = values[i];
      if (values[i] > temp_max) temp_max = values[i];
    }
    minimum = temp_min;
    maximum = temp_max;
    isDrawn = false;
    if (!init_done){
      for (int i=0; i < count; i++){
        values[i] = temp;
      }
      init_done = true;
    }
  }

  unsigned int get_val(int position){
    if (!init_done) return display_height - 1;
    float delta = maximum - minimum;
    float delta_per_pixel;
    if (delta != 0.0){ 
      delta_per_pixel = delta / (display_height - 16);
    }
    else {
      delta_per_pixel = 1;
    }
    float t;
    if (values[position] ==  minimum){
      t = 1;
    }
    else{
      t = (values[position] - minimum) / delta_per_pixel;
    }
    return display_height - round(t);
  }

};

TemperatureArray temperatureArray = TemperatureArray(64);

void readTemperature(void){
  sensors.requestTemperatures(); 
  temperatureArray.add(sensors.getTempCByIndex(0));

  // send data
  _radioData.OnTimeMillis = (int)(sensors.getTempCByIndex(0)*1000);
  if (_radio.send(DESTINATION_RADIO_ID, &_radioData, sizeof(_radioData))) // Note how '&' must be placed in front of the variable name.
  {
      Serial.println("...Success");
      digitalWrite(LEDPIN, LOW);
  }
  else
  {
      Serial.println("...Failed");
      digitalWrite(LEDPIN, HIGH);
      _radioData.FailedTxCount++;
  }
  
}

Task ttemp = Task(*readTemperature, 1000, "temp");

int old_state = true;
bool short_press = false;
bool long_press = false;
unsigned int long_press_threshold = 1500;
unsigned long button_timer = 0;

void readButton() {
  int new_state = digitalRead(BUTTONPIN);
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

bool ledState = false;

void drawYLine(int ypos){
    u8g2.drawHLine(0, ypos, 128);  
}

void setup() {
  Serial.begin(115200);
  
  if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN)){
    Serial.println("Cannot communicate with radio");
    while (1); // Wait here forever.
  }  
  _radioData.FromRadioId = RADIO_ID;
  
  sensors.begin();
  sensors.setResolution(TEMPRESOLUTION);
  
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.setColorIndex(1);
  
  pinMode(BUTTONPIN, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT);
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
      u8g2.print("H " + String(temperatureArray.maximum, 4)); 
      u8g2.setCursor(0, 14);
      u8g2.print("L " + String(temperatureArray.minimum, 4)); 
      u8g2.setCursor(54, 6);
      u8g2.print(String(temperatureArray.values[(temperatureArray.current + 127)%128], 4)); 
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
      u8g2.print(String(duration) + text + " " + String((int)(duration*128)) + text);
      for (int i=0; i<128;i++){
        unsigned int val = temperatureArray.get_val((temperatureArray.current - i + 127)%128);
        u8g2.drawPixel(127 - i, val);
      }
    } while ( u8g2.nextPage() );
    temperatureArray.isDrawn = true;
  }
}
