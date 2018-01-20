
#include <stdio.h>
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "Arduino.h"
#include "LocoNetESP32.h"
#include "oled/SSD1306.h"

/* LocoNet Packet Monitor
  Demonstrates the use of the:

  Loconet switch and sensor processing functions,
  and the use of an I2C attached OLED screen.
*/


LocoNetESP32Class LocoNet;

SSD1306 display(0x3c, 5, 4);
#define BUF_SIZE (1024)

void setup()
{

    // Configure the serial port for 115200 baud
    Serial.begin(115200);
    Serial.println("Loconet ESP 32 Initialising");
    // First initialise the LocoNet interface
    LocoNet.init();
    Serial.println("Display Initialising");
    // Then initialise the display interface */
    display.init();
    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);

}

void loop()
{

    /* don't have to do anything here as the
     * Loconet stuff is all contained in its
     * own thread.
     */

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
}

