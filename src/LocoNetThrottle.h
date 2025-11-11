#pragma once

#include "LocoNet2.h"



#define TH_OP_DEFERRED_SPEED 0x01

typedef enum
{
    TH_ST_FREE = 0,
    TH_ST_IDLE,
    TH_ST_RELEASE,
    TH_ST_ACQUIRE,
    TH_ST_SELECT,
    TH_ST_DISPATCH,
    TH_ST_SLOT_MOVE,
    TH_ST_SLOT_FORCE_FREE,
    TH_ST_SLOT_RESUME,
    TH_ST_SLOT_STEAL,
    TH_ST_IN_USE
} TH_STATE;


typedef enum
{
    TH_SP_ST_28 = 0,  // 000=28 step/ 3 BYTE PKT regular mode
    TH_SP_ST_28_TRI = 1,  // 001=28 step. Generate Trinary packets for this Mobile ADR
    TH_SP_ST_14 = 2,  // 010=14 step MODE
    TH_SP_ST_128 = 3,  // 011=send 128 speed mode packets
    TH_SP_ST_28_ADV = 4,  // 100=28 Step decoder ,Allow Advanced DCC consisting
    TH_SP_ST_128_ADV = 7   // 111=128 Step decoder, Allow Advanced DCC consisting
} TH_SPEED_STEPS;

typedef enum
{
    TH_ER_OK = 0,
    TH_ER_SLOT_IN_USE,
    TH_ER_BUSY,
    TH_ER_NOT_SELECTED,
    TH_ER_NO_LOCO,
    TH_ER_NO_SLOTS
} TH_ERROR;

class LocoNetThrottle
{
public:
    LocoNetThrottle (LocoNetDispatcher *locoNet);

    void init (uint8_t userData, uint8_t options, uint16_t throttleId);
    void processMessage (const lnMsg *LnPacket);
    void process100msActions (void);

    /**
     * Registers a callback for when the address for this throttle changes
     */
    void addressChange (std::function<void (LocoNetThrottle *, uint16_t, uint16_t) > callback)
    {
        addressChangeCallback = callback;
    }
    /**
     * Registers a callback for when the speed for this throttle changes
     */
    void speedChange (std::function<void (LocoNetThrottle *, uint8_t) > callback)
    {
        speedChangeCallback = callback;
    }
    /**
     * Registers a callback for when the direction for this throttle changes
     */
    void directionChange (std::function<void (LocoNetThrottle *, uint8_t) > callback)
    {
        directionChangeCallback = callback;
    }
    /**
     * Registers a callback for when a function for this throttle changes
     */
    void functionChange (std::function<void (LocoNetThrottle *, uint8_t, bool) > callback)
    {
        functionChangeCallback = callback;
    }
    /**
     * Registers a callback for when this throttle changes slots
     */
    void slotStateChange (std::function<void (LocoNetThrottle *, uint8_t) > callback)
    {
        slotStateCallback = callback;
    }
    /**
     * Registers a callback for when a this throttle has an error
     */
    void error (std::function<void (LocoNetThrottle *, TH_ERROR) > callback)
    {
        errorCallback = callback;
    }
    /**
     * Registers a callback for when this throttle changes status
     *                                                             Old Status  New Status
     */
    void throttleStateChange (std::function<void (LocoNetThrottle *, TH_STATE, TH_STATE) > callback)
    {
        stateCallback = callback;
    }
    /**
     * Registers a callback for when this throttle speed steps changes status
     *                                                             Old Status  New Status
     */
    void throttleSpeedStepsChange (std::function<void (LocoNetThrottle *, TH_SPEED_STEPS) > callback)
    {
        speedStepsChangeCallback = callback;
    }

    uint16_t getAddress (void);
    TH_ERROR setAddress (uint16_t Address);
    TH_ERROR stealAddress (uint16_t Address);
    TH_ERROR resumeAddress (uint16_t Address, uint8_t LastSlot);

    TH_ERROR dispatchAddress (void);
    TH_ERROR dispatchAddress (uint16_t Address);


    TH_ERROR acquireAddress (void);
    TH_ERROR idleAddress (void);
    void releaseAddress (void);

    TH_ERROR freeAddress (void);
    TH_ERROR freeAddressForce (uint16_t Address);

    uint8_t getSpeed (void);
    TH_ERROR setSpeed (uint8_t Speed);

    uint8_t getDirection (void);
    TH_ERROR setDirection (uint8_t Direction);

    uint8_t getFunction (uint8_t Function);
    TH_ERROR setFunction (uint8_t Function, uint8_t Value);
    TH_ERROR setDirFunc0to4Direct (uint8_t Value);
    TH_ERROR setFunc5to8Direct (uint8_t Value);

    TH_SPEED_STEPS getSpeedSteps (void);
    void setSpeedSteps (TH_SPEED_STEPS newSpeedSteps);
    const char* getSpeedStepStr (TH_SPEED_STEPS speedStep);

    TH_STATE getState (void);
    const char *getStateStr (TH_STATE State);
    const char *getErrorStr (TH_ERROR Error);
private:
    LocoNetDispatcher * _locoNet;
    TH_STATE _state;                // State of throttle
    uint16_t _ticksSinceLastAction;
    uint16_t _throttleId;           // Id of throttle
    uint8_t _slot;                  // Master Slot index
    uint16_t _address;              // Decoder Address
    uint8_t _speed;                 // Loco Speed
    uint8_t _deferredSpeed;         // Deferred Loco Speed setting
    uint8_t _status1;               // Stat1
    uint8_t _dirFunc0to4;           // Direction
    uint8_t _func5to8;              // Direction
    uint8_t _userData;
    uint8_t _options;
    TH_SPEED_STEPS _speedSteps;

    void updateAddress (uint16_t Address, uint8_t ForceNotify);
    void updateSpeed (uint8_t Speed, uint8_t ForceNotify);
    void updateState (TH_STATE State, uint8_t ForceNotify);
    void updateStatus1 (uint8_t Status, uint8_t ForceNotify);
    void updateDirectionAndFunctions (uint8_t DirFunc0to4, uint8_t ForceNotify);
    void updateFunctions5to8 (uint8_t Func5to8, uint8_t ForceNotify);
    void updateSpeedSteps (TH_SPEED_STEPS SpeedSteps, uint8_t ForceNotify);

    std::function<void (LocoNetThrottle *, uint16_t, uint16_t) > addressChangeCallback;
    std::function<void (LocoNetThrottle *, uint8_t) > speedChangeCallback;
    std::function<void (LocoNetThrottle *, uint8_t, bool) > functionChangeCallback;
    std::function<void (LocoNetThrottle *, uint8_t) > directionChangeCallback;
    std::function<void (LocoNetThrottle *, uint8_t) > slotStateCallback;
    std::function<void (LocoNetThrottle *, TH_STATE, TH_STATE) > stateCallback;
    std::function<void (LocoNetThrottle *, TH_SPEED_STEPS) > speedStepsChangeCallback;
    std::function<void (LocoNetThrottle *, TH_ERROR) > errorCallback;
};