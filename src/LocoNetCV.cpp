#include "LocoNetCVAccess.h"
#include "LocoNetSVCV.h"
#include <Arduino.h>

#define LNCV_SRC_MODULE 0x05
#define LNCV_INTELLIBOX_KPU_DSTL 'I'
#define LNCV_INTELLIBOX_KPU_DSTH 'K'
#define LNCV_REQID_CFGREAD 31
#define LNCV_REQID_CFGWRITE 32
#define LNCV_REQID_CFGREQUEST 33
#define LNCV_FLAG_PRON 0x80
#define LNCV_FLAG_PROFF 0x40

LocoNetCV::LocoNetCV(LocoNet& locoNet) : _locoNet(locoNet) {
    _cvAccess = new LocoNetCVAccess(locoNet);
    _locoNet.onPacket(OPC_IMM_PACKET, std::bind(&LocoNetCV::processMessage, this, std::placeholders::_1));
    _cvAccess->onCvRead(std::bind(&LocoNetCV::cvRead, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    _cvAccess->onCvWrite(std::bind(&LocoNetCV::cvWrite, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

LocoNetCV::~LocoNetCV() {
    delete _cvAccess;
}

void LocoNetCV::processMessage(const lnMsg* rxPacket) {
    lnMsg rxCopy = *rxPacket;
    lnMsg* lnPacket = &rxCopy;

    if (lnPacket->ub.mesg_size == 15) {
        _cvAccess->computeBytesFromPxct(lnPacket->ub);
        if (lnPacket->ub.ReqId == LNCV_REQID_CFGREQUEST) {
            if (lnPacket->ub.payload.data.deviceClass == 0xFFFF && lnPacket->ub.payload.data.lncvNumber == 0x0000 && lnPacket->ub.payload.data.lncvValue == 0xFFFF) {
                if (_discoveryCallback) {
                    uint16_t artNr = 0;
                    uint16_t modAddr = 0;
                    if (_discoveryCallback(artNr, modAddr) == 0) {
                        lnMsg response;
                        _cvAccess->makeLncvResponse(response.ub, lnPacket->ub.SRC, artNr, 0x00, modAddr, 0x00);
                        _locoNet.send(&response);
                    }
                }
            } else if (lnPacket->ub.payload.data.flags == LNCV_FLAG_PRON) {
                if (_progStartCallback) {
                    uint16_t artNr = 0;
                    uint16_t modAddr = 0;
                    if (_progStartCallback(artNr, modAddr) == 0) {
                        lnMsg response;
                        _cvAccess->makeLncvResponse(response.ub, lnPacket->ub.SRC, artNr, 0x00, modAddr, 0x80);
                        _locoNet.send(&response);
                    }
                }
            } else if (lnPacket->ub.payload.data.flags == LNCV_FLAG_PROFF) {
                if (_progStopCallback) {
                    _progStopCallback(lnPacket->ub.payload.data.deviceClass, lnPacket->ub.payload.data.lncvValue);
                }
            }
        }
    }
}

int8_t LocoNetCV::cvRead(uint16_t dev, uint16_t cv, uint16_t& val) {
    if (_cvReadCallback) {
        return _cvReadCallback(dev, cv, val);
    }
    return -1;
}

int8_t LocoNetCV::cvWrite(uint16_t dev, uint16_t cv, uint16_t val) {
    if (_cvWriteCallback) {
        return _cvWriteCallback(dev, cv, val);
    }
    return -1;
}

void LocoNetCV::onDiscoveryRequest(std::function<int8_t(uint16_t&, uint16_t&)> callback) {
    _discoveryCallback = callback;
}

void LocoNetCV::onProgrammingStart(std::function<int8_t(uint16_t&, uint16_t&)> callback) {
    _progStartCallback = callback;
}

void LocoNetCV::onProgrammingStop(std::function<int8_t(uint16_t, uint16_t)> callback) {
    _progStopCallback = callback;
}

void LocoNetCV::onCVRead(std::function<int8_t(uint16_t, uint16_t, uint16_t&)> callback) {
    _cvReadCallback = callback;
}

void LocoNetCV::onCVWrite(std::function<int8_t(uint16_t, uint16_t, uint16_t)> callback) {
    _cvWriteCallback = callback;
}
