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
#include <Arduino.h>
#include <LocoNetESP32.h>
#include <SSD1306.h>

LocoNetESP32 LocoNet;
SSD1306 display(0x3c, 5, 4);
#define BUF_SIZE (1024)

void setup()
{

    // Configure the serial port for 57600 baud
    Serial.begin(115200);

    Serial.println("LocoNet Monitor");

    // First initialize the LocoNet interface

    LocoNet.init();
    LocoNet.onPacket(0xFF, [](lnMsg *rxPacket) {
        Serial.print("rx'd ");
        for(uint8_t x = 0; x < 4; x++) {
            uint8_t val = rxPacket->data[x];
            // Print a leading 0 if less than 16 to make 2 HEX digits
            if(val < 16) {
                Serial.print('0');
            }

            Serial.print(val, HEX);
            Serial.print(' ');
        }
        Serial.print("\r\n");
        return false;
    });
    LocoNet.onPacket(OPC_SW_REQ, [](lnMsg *lnPacket) {
        uint16_t address = (lnPacket->srq.sw1 | ( ( lnPacket->srq.sw2 & 0x0F ) << 7 )) + 1;
        bool output = lnPacket->srq.sw2 & OPC_SW_REQ_OUT;
        bool direction = lnPacket->srq.sw2 & OPC_SW_REQ_DIR;
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(0, 0, "Switch Request: ");
        display.drawString(0, 20, String(address) + direction ? ":Closed" : ":Thrown");
        display.drawString(0, 40, output ? "On" : "Off");
        display.display();

        Serial.print("Switch Request: ");
        Serial.print(address, DEC);
        Serial.print(':');
        Serial.print(direction ? "Closed" : "Thrown");
        Serial.print(" - ");
        Serial.println(output ? "On" : "Off");
        return true;
    });
    LocoNet.onPacket(OPC_SW_REP, [](lnMsg *lnPacket) {
        uint16_t address = (lnPacket->srq.sw1 | ( ( lnPacket->srq.sw2 & 0x0F ) << 7 )) + 1;
        bool state = lnPacket->srq.sw2 & OPC_SW_REP_HI;
        bool sensor = lnPacket->srq.sw2 & OPC_SW_REP_SW;
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(0, 0, "Switch Sensor Report: ");
        display.drawString(0, 20, String(address) + sensor ? ":Switch" : ":Aux");
        display.drawString(0, 40, state ? "Active" : "Inactive");
        display.display();

        Serial.print("Switch Sensor Report: ");
        Serial.print(address, DEC);
        Serial.print(':');
        Serial.print(sensor ? "Switch" : "Aux");
        Serial.print(" - ");
        Serial.println(state ? "Active" : "Inactive");
        return true;
    });
    LocoNet.onPacket(OPC_INPUT_REP, [](lnMsg *lnPacket) {
        uint16_t address = (lnPacket->srq.sw1 | ( ( lnPacket->srq.sw2 & 0x0F ) << 7 ));
        address <<= 1;
        address += ( lnPacket->ir.in2 & OPC_INPUT_REP_SW ) ? 2 : 1 ;
        bool state = lnPacket->ir.in2 & OPC_INPUT_REP_HI;
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.drawString(0, 0, "Sensor: ");
        display.drawString(0, 20, String(address));
        display.drawString(0, 40, state ? "Active" : "Inactive");
        display.display();
        Serial.print("Sensor: ");
        Serial.print(address, DEC);
        Serial.print(" - ");
        Serial.println(state ? "Active" : "Inactive");
        return true;
    });

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