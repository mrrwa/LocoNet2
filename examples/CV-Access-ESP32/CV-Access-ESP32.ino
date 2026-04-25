#include <LocoNetStreamESP32.h>
#include <LocoNetCVAccess.h>

LocoNetBus bus;
LocoNetDispatcher parser(&bus);
LocoNetStreamESP32 lnStream(1, 33, 32, true, true, &bus);
LocoNetCVAccess cvAccess(parser);

uint16_t cvValues[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

int8_t cvRead(uint16_t dev, uint16_t cv, uint16_t& val) {
    if (cv < 10) {
        val = cvValues[cv];
        return 0;
    }
    return -1;
}

int8_t cvWrite(uint16_t dev, uint16_t cv, uint16_t val) {
    if (cv < 10) {
        cvValues[cv] = val;
        return 0;
    }
    return -1;
}

void setup() {
    Serial.begin(115200);
    Serial.println("LocoNet CV Access Demo");

    lnStream.start();

    cvAccess.onCvRead(cvRead);
    cvAccess.onCvWrite(cvWrite);

    parser.onPacket(CALLBACK_FOR_ALL_OPCODES, [](const lnMsg* rxPacket) {
        char tmp[100];
        formatMsg(*rxPacket, tmp, sizeof(tmp));
        Serial.printf("onPacket: %s\n", tmp);
    });
}

void loop() {
    lnStream.process();
    delay(10);
}
