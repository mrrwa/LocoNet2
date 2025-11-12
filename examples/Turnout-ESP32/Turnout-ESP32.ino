#include <Arduino.h>
#include <LocoNetESP32.h>
#include <LocoNetTurnout.h>

// Change this to the GPIO pin you have connected to the LocoNet bus
const int loconetRxPin = 16;
const int loconetTxPin = 17;

LocoNetESP32 loconet(loconetRxPin, loconetTxPin);
LocoNetTurnout turnout(&loconet);

void setup() {
    Serial.begin(115200);
    Serial.println("LocoNet Turnout Example");

    loconet.init();
    turnout.setAddress(1); // Set the turnout address to 1
}

void loop() {
    Serial.println("Setting turnout to thrown");
    turnout.setState(true);
    delay(2000);

    Serial.println("Setting turnout to closed");
    turnout.setState(false);
    delay(2000);
}
