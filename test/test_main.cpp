#include <Arduino.h>
#include <unity.h>
#include "LocoNet2.h"
#include "LocoNetCVAccess.h"

// Include implementation files to resolve linker errors
#include "LocoNet2.cpp"
#include "LocoNetCVAccess.cpp"
#include "LocoNetMessageBuffer.cpp"


uint16_t cvValues[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
bool readCallbackCalled = false;
bool writeCallbackCalled = false;

int8_t cvRead(uint16_t dev, uint16_t cv, uint16_t& val) {
    if (cv < 10) {
        val = cvValues[cv];
        readCallbackCalled = true;
        return 0;
    }
    return -1;
}

int8_t cvWrite(uint16_t dev, uint16_t cv, uint16_t val) {
    if (cv < 10) {
        cvValues[cv] = val;
        writeCallbackCalled = true;
        return 0;
    }
    return -1;
}

void test_cv_read_callback() {
    LocoNetBus bus;
    LocoNetDispatcher parser(&bus);
    LocoNetCVAccess cvAccess(parser);
    cvAccess.onCvRead(cvRead);

    lnMsg msg;
    msg.ub.command = OPC_PEER_XFER;
    msg.ub.mesg_size = 15;
    msg.ub.ReqId = 31; // LNCV_REQID_CFGREAD
    msg.ub.payload.data.deviceClass = 1;
    msg.ub.payload.data.lncvNumber = 1;

    parser.onMessage(msg); // Simulate receiving a message

    TEST_ASSERT_TRUE(readCallbackCalled);
}

void test_cv_write_callback() {
    LocoNetBus bus;
    LocoNetDispatcher parser(&bus);
    LocoNetCVAccess cvAccess(parser);
    cvAccess.onCvWrite(cvWrite);

    lnMsg msg;
    msg.ub.command = OPC_PEER_XFER;
    msg.ub.mesg_size = 15;
    msg.ub.ReqId = 32; // LNCV_REQID_CFGWRITE
    msg.ub.payload.data.deviceClass = 1;
    msg.ub.payload.data.lncvNumber = 1;
    msg.ub.payload.data.lncvValue = 42;

    parser.onMessage(msg); // Simulate receiving a message

    TEST_ASSERT_TRUE(writeCallbackCalled);
    TEST_ASSERT_EQUAL(42, cvValues[1]);
}


void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_cv_read_callback);
    RUN_TEST(test_cv_write_callback);
    UNITY_END();
}

void loop() {
}
