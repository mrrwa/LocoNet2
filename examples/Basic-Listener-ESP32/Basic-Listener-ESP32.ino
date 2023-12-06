#if defined(ARDUINO_M5Stick_C)
#include <M5StickC.h>
#endif

#include <LocoNetStreamESP32.h>

LocoNetBus bus;

#define LOCONET_PIN_RX 33
#define LOCONET_PIN_TX 32
#define LOCONET_UART_SIGNAL_INVERT  true

#include <LocoNetStream.h>
LocoNetDispatcher parser(&bus);
LocoNetStreamESP32 lnStream(1, LOCONET_PIN_RX, LOCONET_PIN_TX, LOCONET_UART_SIGNAL_INVERT, &bus);

void setup() {
#if defined(ARDUINO_M5Stick_C)
  M5.begin();
#endif
  
  Serial.begin(115200);
  Serial.println("LocoNet2 Basic Demo");

  // Setup Serial1 to work with M5StickC Grove Port and invert the signal to work with IoTTStick LocoNet Comms Unit.
  lnStream.start();

  parser.onPacket(CALLBACK_FOR_ALL_OPCODES, [](const lnMsg *rxPacket) {
      char tmp[100];
      formatMsg(*rxPacket, tmp, sizeof(tmp));
      Serial.printf("onPacket: %s\n", tmp);
  });

  parser.onSwitchRequest([](uint16_t address, bool output, bool direction) {
      Serial.print("Switch Request: ");
      Serial.print(address, DEC);
      Serial.print(':');
      Serial.print(direction ? "Closed" : "Thrown");
      Serial.print(" - ");
      Serial.println(output ? "On" : "Off");
  });
  parser.onSwitchReport([](uint16_t address, bool state, bool sensor) {
      Serial.print("Switch/Sensor Report: ");
      Serial.print(address, DEC);
      Serial.print(':');
      Serial.print(sensor ? "Switch" : "Aux");
      Serial.print(" - ");
      Serial.println(state ? "Active" : "Inactive");
  });
  parser.onSensorChange([](uint16_t address, bool state) {
      Serial.print("Sensor: ");
      Serial.print(address, DEC);
      Serial.print(" - ");
      Serial.println(state ? "Active" : "Inactive");
  });
}

uint16_t sensorIndex = 0;

void loop() {
  lnStream.process();
  
  delay(10);
#if defined(ARDUINO_M5Stick_C)
  M5.update();
  if(M5.BtnA.wasPressed())
  {
    reportSensor(&bus, sensorIndex++, 0);
  }

  if(M5.BtnB.wasPressed())
  {
//    reportPower(&bus, (sensorIndex % 2));
    parser.send(0xA0, 0x03, 0x00);
  }
#endif  
}
