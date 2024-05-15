#include <LocoNetStreamSTM32.h>
#include <Bounce2.h>

#define LOCONET_PIN_RX  0
#define LOCONET_PIN_TX  1

#define BUTTON_PIN      7 


LocoNetBus bus;
LocoNetDispatcher parser(&bus);

// The line below initialised the LocoNet interface for the correct signal polarity of the IoTT LocoNet Interface board
// See: https://myiott.org/index.php/iott-stick/communication-modules/loconet-interface
LocoNetStreamSTM32 lnStream(&Serial, 0, 1, &bus);

Bounce2::Button button = Bounce2::Button();

void log_printf( const char * format, ... )
{
  char buffer[256];
  va_list args;
  va_start (args, format);
  vsprintf (buffer,format, args);
  Serial.print(buffer);
  va_end (args);
}

void setup()
{
  Serial.begin(115200);
  while((!Serial) && (millis() < 5000))
    delay(1);

  printf("String from Printf\n");

  Serial.println("LocoNet2 Basic Demo");
  Serial.println("setup: before lnStream.start()");
  lnStream.start();
  Serial.println("setup: after lnStream.start()");

  parser.onPacket(CALLBACK_FOR_ALL_OPCODES, [](const lnMsg *rxPacket) {
      char tmp[100];
      formatMsg(*rxPacket, tmp, sizeof(tmp));
      Serial.print("onPacket: ");
      Serial.println(tmp);
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
