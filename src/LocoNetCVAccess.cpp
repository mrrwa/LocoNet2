#include "LocoNetCVAccess.h"
#include "LocoNet2.h"
#include <Arduino.h>

#define LNCV_SRC_MODULE 0x05
#define LNCV_INTELLIBOX_KPU_DSTL 'I'
#define LNCV_INTELLIBOX_KPU_DSTH 'K'
#define LNCV_REQID_CFGREAD 31
#define LNCV_REQID_CFGWRITE 32
#define LNCV_REQID_CFGREQUEST 33

LocoNetCVAccess::LocoNetCVAccess(LocoNet& locoNet) : _locoNet(locoNet) {
    _locoNet.onPacket(OPC_IMM_PACKET, std::bind(&LocoNetCVAccess::processMessage, this, std::placeholders::_1));
    _locoNet.onPacket(OPC_PEER_XFER, std::bind(&LocoNetCVAccess::processMessage, this, std::placeholders::_1));
}

void LocoNetCVAccess::onCvRead(std::function<int8_t(uint16_t, uint16_t, uint16_t&)> callback) {
    _cvReadCallback = callback;
}

void LocoNetCVAccess::onCvWrite(std::function<int8_t(uint16_t, uint16_t, uint16_t)> callback) {
    _cvWriteCallback = callback;
}

void LocoNetCVAccess::processMessage(const lnMsg* rxPacket) {
    lnMsg rxCopy = *rxPacket;
    lnMsg* lnPacket = &rxCopy;

    if (lnPacket->ub.mesg_size == 15) {
        computeBytesFromPxct(lnPacket->ub);
        lnMsg response;

        switch (lnPacket->ub.ReqId) {
            case LNCV_REQID_CFGREAD:
                if (_cvReadCallback) {
                    uint16_t val = 0;
                    int8_t ret = _cvReadCallback(lnPacket->ub.payload.data.deviceClass, lnPacket->ub.payload.data.lncvNumber, val);
                    if (ret == 0) {
                        makeLncvResponse(response.ub, lnPacket->ub.SRC, lnPacket->ub.payload.data.deviceClass, lnPacket->ub.payload.data.lncvNumber, val, 0);
                        _locoNet.send(&response);
                    } else if (ret > 0) {
                        _locoNet.send(OPC_LONG_ACK, (0x7F & lnPacket->ub.command), ret);
                    }
                }
                break;
            case LNCV_REQID_CFGWRITE:
                if (_cvWriteCallback) {
                    int8_t ret = _cvWriteCallback(lnPacket->ub.payload.data.deviceClass, lnPacket->ub.payload.data.lncvNumber, lnPacket->ub.payload.data.lncvValue);
                    if (ret >= 0) {
                        _locoNet.send(OPC_LONG_ACK, (0x7F & lnPacket->ub.command), ret);
                    }
                }
                break;
        }
    }
}

void LocoNetCVAccess::makeLncvResponse(UhlenbrockMsg& ub, uint8_t src, uint16_t dev, uint16_t cv, uint16_t val, uint8_t flags) {
    ub.command = OPC_PEER_XFER;
    ub.mesg_size = 15;
    ub.SRC = LNCV_SRC_MODULE;
    ub.DSTL = src;
    ub.DSTH = 0x00;
    ub.ReqId = LNCV_REQID_CFGREAD;
    ub.PXCT1 = 0x00;
    ub.payload.data.deviceClass = dev;
    ub.payload.data.lncvNumber = cv;
    ub.payload.data.lncvValue = val;
    ub.payload.data.flags = flags;
    computePxctFromBytes(ub);
}

void LocoNetCVAccess::computeBytesFromPxct(UhlenbrockMsg& ub) {
    uint8_t mask = 0x01;
    for (int i = 0; i < 7; ++i) {
        if ((ub.PXCT1 & mask) != 0x00) {
            ub.payload.D[i] |= 0x80;
        }
        mask <<= 1;
    }
    ub.PXCT1 = 0x00;
}

void LocoNetCVAccess::computePxctFromBytes(UhlenbrockMsg& ub) {
    uint8_t mask = 0x01;
    ub.PXCT1 = 0x00;
    for (int i = 0; i < 7; ++i) {
        if ((ub.payload.D[i] & 0x80) != 0x00) {
            ub.PXCT1 |= mask;
            ub.payload.D[i] &= 0x7F;
        }
        mask <<= 1;
    }
}
