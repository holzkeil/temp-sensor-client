Arduino Temperature Device

Running on a Pro Mini 5V. Is use in rooms to monitor the temperature.

## Features ##

* Measuring the temperature
* Plotting the temperature on a display
* Writing high, low and current values on a display
* Auto scaling the graph
* Adjusting the measurment interval via button
* Writing measurement interval on a display 
* Writing last used interval to EEPROM, reading on startup
* Sending the temperature to another arduino or raspi via nrf2401l
* Indicating sending success on an LED
* Running multiple functions in "parallel" with a Task class
* Setting various options
* Many more...

## Images ##

[Overview](images/overview.jpg)
[Graph](images/graph.jpg)
[Settings](images/setting.jpg)



## Part List ##

* Arduino
  * I have used a Pro Mini 5V
  * a Nano was also working
  * and a Uno R3
* Tactile Button
  * but no resistor since I use the internal pull ups and reverse logic
* Indicator LED
* Resistor for LED, e.g. 680 Ohms to keep the current low
* OLED
  * I have used a generic SSD1306 based one with SPI
  * SH1106 based one with I2C was also working but then you are very tight on RAM
* NRF24L01 Transceiver
  * NRF24L01 5V to 3.3V adapter board, my Pro Mini has no native 3.3V output so I need one of these
  * 100nF capacitor added between 3.3V and GND on the NRF24L01 to stabelize voltage, was not working without
* Wires

## Memory Usage ##

### According to Arduino IDE 1.8.10 ###

Sketch uses 25128 bytes (81%) of program storage space. Maximum is 30720 bytes.
Global variables use 1630 bytes (79%) of dynamic memory, leaving 418 bytes for local variables. Maximum is 2048 bytes.

### According to Visual Studio Code 1.43.1 with Platform IO Core 4.3.0b2 Home 3.1.1 ###

RAM:   [========  ]  81.1% (used 1660 bytes from 2048 bytes)
Flash: [========  ]  81.8% (used 25114 bytes from 30720 bytes)

## Contribution ##

Pull requests are welcome, but keep them focused on one feature, bugfix or other improvement.

## Schematics ##

...also...

## 3D models ##

...guess what...