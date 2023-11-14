#include <LocoNetStreamRP2040.h>
#include <Bounce2.h>

#define LOCONET_PIN_RX  17
#define LOCONET_PIN_TX  16

#define BUTTON_PIN      18 


LocoNetBus bus;
LocoNetDispatcher parser(&bus);
LocoNetStreamRP2040 lnStream(&Serial1, LOCONET_PIN_RX, LOCONET_PIN_TX, &bus);
Bounce2::Button button = Bounce2::Button();

void LocoNetActiveInterrupt(void)
{
  lnStream.handleLocoNetActivityInterrupt();
}

void setup()
{
  Serial.begin(115200);
  while((!Serial) && (millis() < 5000))
    delay(1);

  Serial.println("LocoNet2 Basic Demo");
  Serial.println("setup: before lnStream.start()");
  lnStream.start();
  Serial.println("setup: after lnStream.start()");

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

  button.attach (BUTTON_PIN,INPUT_PULLUP);
  button.interval(25);
  button.setPressedState(LOW);

  attachInterrupt(LOCONET_PIN_RX, LocoNetActiveInterrupt, FALLING);
  pinMode(LED_BUILTIN, OUTPUT); 
}

uint16_t sensorIndex = 0;

void loop()
{
  lnStream.process();

  digitalWrite(LED_BUILTIN, lnStream.isBusy()); 

  button.update();
  if(button.pressed())
  {
    reportSensor(&bus, sensorIndex++, 0);
  }

}
