// LocoNet Packet Monitor
// Demonstrates the use of the:
//
//   LocoNet.processSwitchSensorMessage(LnPacket)
//
//   Requires the ESP8266 and ESP32 SSD1306 OLED
//   library by Daniel Eichhorn (@squix78) and
//   Fabrice Weinberg (@FWeinb)
//   https://github.com/squix78/esp8266-oled-ssd1306
//
// function and examples of each of the notifyXXXXXXX user call-back functions
#include <stdio.h>
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "Arduino.h"
#include <LocoNetESP32.h>
#include <SSD1306.h>



LocoNetESP32Class LocoNet;
SSD1306 display(0x3c, 5, 4);
#define BUF_SIZE (1024)

void setup()
{

    // Configure the serial port for 57600 baud
    Serial.begin(115200);

    Serial.println("LocoNet Monitor");

    // First initialize the LocoNet interface

    LocoNet.init();

    Serial.println("Display Initialising");
    // Then initialise the display interface */
    display.init();
    display.flipScreenVertically();
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Loconet Monitor");
    display.display();

}

void loop()
{

    delay(1000);

}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Sensor messages
void notifySensor(uint16_t Address, uint8_t State)
{
    Serial.print("Sensor: ");
    Serial.print(Address, DEC);
    Serial.print(" - ");
    Serial.println(State ? "Active" : "Inactive");
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Request messages
void notifySwitchRequest(uint16_t Address, uint8_t Output, uint8_t Direction)
{
    char tmp[24];
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    sprintf(tmp, "Switch Request: ");
    display.drawString(0, 0, tmp);
    sprintf(tmp, "%d:%s", Address, (Direction ? "Closed" : "Thrown"));
    display.drawString(0, 20, tmp);
    sprintf(tmp, "%s", (Output ? "On" : "Off"));
    display.drawString(0, 40, tmp);
    display.display();

    Serial.print("Switch Request: ");
    Serial.print(Address, DEC);
    Serial.print(':');
    Serial.print(Direction ? "Closed" : "Thrown");
    Serial.print(" - ");
    Serial.println(Output ? "On" : "Off");
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Output Report messages
void notifySwitchOutputsReport(uint16_t Address, uint8_t ClosedOutput, uint8_t ThrownOutput)
{
    char tmp[24];
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    sprintf(tmp, "Switch Output Repor: ");
    display.drawString(0, 0, tmp);
    sprintf(tmp, "%d:%s", Address, (ClosedOutput ? "On" : "Off"));
    display.drawString(0, 20, tmp);
    sprintf(tmp, "%s", (ThrownOutput ? "On" : "Off"));
    display.drawString(0, 40, tmp);
    display.display();

    Serial.print("Switch Outputs Report: ");
    Serial.print(Address, DEC);
    Serial.print(": Closed - ");
    Serial.print(ClosedOutput ? "On" : "Off");
    Serial.print(": Thrown - ");
    Serial.println(ThrownOutput ? "On" : "Off");
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch Sensor Report messages
void notifySwitchReport(uint16_t Address, uint8_t State, uint8_t Sensor)
{

    char tmp[24];
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    sprintf(tmp, "Switch Sensor Report: ");
    display.drawString(0, 0, tmp);
    sprintf(tmp, "%d:%s", Address, (Sensor ? "Switch" : "Aux"));
    display.drawString(0, 20, tmp);
    sprintf(tmp, "%s", (State ? "Active" : "Inactive"));
    display.drawString(0, 40, tmp);
    display.display();

    Serial.print("Switch Sensor Report: ");
    Serial.print(Address, DEC);
    Serial.print(':');
    Serial.print(Sensor ? "Switch" : "Aux");
    Serial.print(" - ");
    Serial.println(State ? "Active" : "Inactive");
}

// This call-back function is called from LocoNet.processSwitchSensorMessage
// for all Switch State messages
void notifySwitchState(uint16_t Address, uint8_t Output, uint8_t Direction)
{
    char tmp[24];
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    sprintf(tmp, "Switch State: ");
    display.drawString(0, 0, tmp);
    sprintf(tmp, "%d:%s", Address, (Direction ? "Closed" : "Thrown"));
    display.drawString(0, 20, tmp);
    sprintf(tmp, "%s", (Output ? "On" : "Off"));
    display.drawString(0, 40, tmp);
    display.display();

    Serial.print("Switch State: ");
    Serial.print(Address, DEC);
    Serial.print(':');
    Serial.print(Direction ? "Closed" : "Thrown");
    Serial.print(" - ");
    Serial.println(Output ? "On" : "Off");
}

void notifyMessage(lnMsg *rxPacket)
{
    Serial.print("rx'd ");
    for(uint8_t x = 0; x < 4; x++)
    {
        uint8_t val = rxPacket->data[x];
        // Print a leading 0 if less than 16 to make 2 HEX digits
        if(val < 16)
        {
            Serial.print('0');
        }

        Serial.print(val, HEX);
        Serial.print(' ');
    }
    Serial.print("\r\n");
}
