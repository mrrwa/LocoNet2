#pragma once

#include "LocoNet2.h"
#include "LocoNetCVAccess.h"

/************************************************************************************
 SV (System Variable Handling
 ************************************************************************************/

typedef enum
{
    SV_EE_SZ_256 = 0, SV_EE_SZ_512 = 1, SV_EE_SZ_1024 = 2, SV_EE_SZ_2048 = 3, SV_EE_SZ_4096 = 4, SV_EE_SZ_8192 = 5
} SV_EE_SIZE;

typedef enum
{
    SV_WRITE_SINGLE = 0x01,
    SV_READ_SINGLE = 0x02,
    SV_WRITE_MASKED = 0x03,
    SV_WRITE_QUAD = 0x05,
    SV_READ_QUAD = 0x06,
    SV_DISCOVER = 0x07,
    SV_IDENTIFY = 0x08,
    SV_CHANGE_ADDRESS = 0x09,
    SV_RECONFIGURE = 0x0F
} SV_CMD;

typedef enum
{
    SV_ADDR_EEPROM_SIZE = 1,
    SV_ADDR_SW_VERSION = 2,
    SV_ADDR_NODE_ID_L = 3,
    SV_ADDR_NODE_ID_H = 4,
    SV_ADDR_SERIAL_NUMBER_L = 5,
    SV_ADDR_SERIAL_NUMBER_H = 6,
    SV_ADDR_USER_BASE = 7,
} SV_ADDR;

typedef enum
{
    SV_NOT_CONSUMED = 0, SV_CONSUMED_OK = 1, SV_ERROR = 2, SV_DEFERRED_PROCESSING_NEEDED = 3
} SV_STATUS;

#define SV_MANUFACTURER_DIY		13

class LocoNetSystemVariable
{
public:
    LocoNetSystemVariable (LocoNet &locoNet, uint8_t newMfgId, uint8_t newDevId, uint16_t newProductId,
                           uint8_t newSwVersion);

    SV_STATUS processMessage (const lnMsg *LnPacket);
    SV_STATUS doDeferredProcessing (void);
    void onSVChange (std::function<void (uint16_t, uint8_t, uint8_t) > callback)
    {
        _svChangeCallback = callback;
    }
    void reconfigureCallback (std::function<void() > callback)
    {
        _reconfigureCallback = callback;
    }
private:
    LocoNet &_locoNet;
    uint8_t _mfgId;
    uint8_t _devId;
    uint16_t _productId;
    uint8_t _swVersion;
    bool _deferredProcessingRequired;
    uint8_t _deferredSrcAddr;
    std::function<void (uint16_t, uint8_t, uint8_t) > _svChangeCallback;
    std::function<void() > _reconfigureCallback;


    uint8_t readSVStorage (uint16_t Offset);
    uint8_t writeSVStorage (uint16_t Offset, uint8_t Value);
    uint8_t isSVStorageValid (uint16_t Offset);
    uint16_t readSVNodeId (void);
    uint16_t writeSVNodeId (uint16_t newNodeId);
    bool CheckAddressRange (uint16_t startAddress, uint8_t Count);
    void reconfigure();
};

class LocoNetCV {
public:
    LocoNetCV(LocoNet& locoNet);
    ~LocoNetCV();

    void onDiscoveryRequest(std::function<int8_t(uint16_t&, uint16_t&)> callback);
    void onProgrammingStart(std::function<int8_t(uint16_t&, uint16_t&)> callback);
    void onProgrammingStop(std::function<int8_t(uint16_t, uint16_t)> callback);
    void onCVRead(std::function<int8_t(uint16_t, uint16_t, uint16_t&)> callback);
    void onCVWrite(std::function<int8_t(uint16_t, uint16_t, uint16_t)> callback);

private:
    void processMessage(const lnMsg* rxPacket);
    int8_t cvRead(uint16_t dev, uint16_t cv, uint16_t& val);
    int8_t cvWrite(uint16_t dev, uint16_t cv, uint16_t val);

    LocoNet& _locoNet;
    LocoNetCVAccess* _cvAccess;
    std::function<int8_t(uint16_t&, uint16_t&)> _discoveryCallback;
    std::function<int8_t(uint16_t&, uint16_t&)> _progStartCallback;
    std::function<int8_t(uint16_t, uint16_t)> _progStopCallback;
    std::function<int8_t(uint16_t, uint16_t, uint16_t&)> _cvReadCallback;
    std::function<int8_t(uint16_t, uint16_t, uint16_t)> _cvWriteCallback;
};
