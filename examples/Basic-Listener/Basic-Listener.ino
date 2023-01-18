#if defined(ARDUINO_M5Stick_C)
#include <M5StickC.h>
#endif

#include <LocoNet2.h>
#include "esp32-hal.h"

LocoNetBus bus;

#define LOCONET_PIN_RX 33
#define LOCONET_PIN_TX 32
#define LOCONET_UART_SIGNAL_INVERT  true

#include <LocoNetStream.h>
LocoNetStream locoNetPhy(&bus);

LocoNetDispatcher parser(&bus);


#include "soc/uart_struct.h"
#include "esp32-hal.h"

// The following line is fron the ESP32 SDK file: Arduino15/packages/esp32/hardware/esp32/2.0.4/tools/sdk/esp32/include/hal/esp32/include/hal/uart_ll.h
#if defined(ARDUINO_M5Stick_C)
  #define UART_LL_GET_HW(num) (((num) == 0) ? (&UART0) : (((num) == 1) ? (&UART1) : (&UART2)))
  
#elif defined(ARDUINO_ESP32C3_DEV)
  #define UART_LL_GET_HW(num) (((num) == 0) ? (&UART0) : (&UART1))
#endif

// The class below provides access to the protected members in the HardwareSerial class needed to perform required low-level UART operations
class LocoNetHardwareSerial:public HardwareSerial
{
  public:
    LocoNetHardwareSerial(int uart_nr) : HardwareSerial(uart_nr){};
    
    bool isLocoNetActive(void)
    {
      uart_dev_t *hw = UART_LL_GET_HW(_uart_nr);
#if defined(ARDUINO_M5Stick_C)      
      return hw->status.st_urx_out != 0;
#elif defined(ARDUINO_ESP32C3_DEV)
      return hw->fsm_status.st_urx_out != 0;
#endif
    };
    
    uint32_t updateRxFifoFullThreshold(uint32_t newThreshold)
    {
      uart_dev_t *hw = UART_LL_GET_HW(_uart_nr);

      uint32_t oldThreshold = hw->conf1.rxfifo_full_thrhd;
      hw->conf1.rxfifo_full_thrhd = newThreshold;
      
      return oldThreshold;
    };
};

LocoNetHardwareSerial lnSerial(1);

uint32_t LocoNetUpdateRxFifoFullThreshold(uint32_t newThreshold)
{
  return lnSerial.updateRxFifoFullThreshold(newThreshold);
}

bool LocoNetIsBusy(void)
{
  return lnSerial.isLocoNetActive();
}

  // Send a LocoNet BREAK by changign the BAUD to a slower speed and send a 0 (zero) 
void LocoNetSendBreak(void)
{
  lnSerial.updateBaudRate(LOCONET_BREAK_BAUD);
  lnSerial.write((uint8_t) 0);
  lnSerial.flush();
  lnSerial.updateBaudRate(LOCONET_BAUD);
}

void setup() {
#if defined(ARDUINO_M5Stick_C)
  M5.begin();
#endif
  
  Serial.begin(115200);
  Serial.println("LocoNet2 Basic Demo");

  // Setup Serial1 to work with M5StickC Grove Port and invert the signal to work with IoTTStick LocoNet Comms Unit.
  lnSerial.begin(LOCONET_BAUD, SERIAL_8N1, LOCONET_PIN_RX, LOCONET_PIN_TX, LOCONET_UART_SIGNAL_INVERT);
  locoNetPhy.begin(lnSerial, LocoNetIsBusy, LocoNetSendBreak, LocoNetUpdateRxFifoFullThreshold);

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
  locoNetPhy.process();
  
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
