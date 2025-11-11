#pragma once

#include <functional>
#include "ln_opc.h"

#define LNCV_MIN_MODULE_ADDR 0
#define LNCV_MAX_MODULE_ADDR 65534

class LocoNetDispatcher;
using LocoNet = LocoNetDispatcher;

class LocoNetCVAccess {
public:
    LocoNetCVAccess(LocoNet& locoNet);

    void onCvRead(std::function<int8_t(uint16_t, uint16_t, uint16_t&)> callback);
    void onCvWrite(std::function<int8_t(uint16_t, uint16_t, uint16_t)> callback);

    void makeLncvResponse(UhlenbrockMsg& ub, uint8_t src, uint16_t dev, uint16_t cv, uint16_t val, uint8_t flags);
    void computePxctFromBytes(UhlenbrockMsg& ub);
    void computeBytesFromPxct(UhlenbrockMsg& ub);

private:
    void processMessage(const lnMsg* msg);

    LocoNet& _locoNet;
    std::function<int8_t(uint16_t, uint16_t, uint16_t&)> _cvReadCallback;
    std::function<int8_t(uint16_t, uint16_t, uint16_t)> _cvWriteCallback;
};
