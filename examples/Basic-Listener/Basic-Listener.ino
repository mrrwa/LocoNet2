#include <LocoNetESP32.h>

LocoNetBus bus;
#define LOCONET_PIN_RX 16
#define LOCONET_PIN_TX 17

LocoNetESP32 locoNetPhy(&bus, LOCONET_PIN_RX, LOCONET_PIN_TX, 0);
LocoNetDispatcher parser(&bus);

void setup() {

}

void loop() {

}
