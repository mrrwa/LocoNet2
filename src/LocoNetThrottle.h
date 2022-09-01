#pragma once

#include "LocoNet2.h"



#define TH_OP_DEFERRED_SPEED 0x01

typedef enum {
    TH_ST_FREE = 0,
    TH_ST_ACQUIRE,
    TH_ST_SELECT,
    TH_ST_DISPATCH,
    TH_ST_SLOT_MOVE,
    TH_ST_SLOT_FREE,
    TH_ST_SLOT_RESUME,
    TH_ST_IN_USE
} TH_STATE;

typedef enum {
    TH_ER_OK = 0, TH_ER_SLOT_IN_USE, TH_ER_BUSY, TH_ER_NOT_SELECTED, TH_ER_NO_LOCO, TH_ER_NO_SLOTS
} TH_ERROR;

class LocoNetThrottle {
    public:
        LocoNetThrottle(LocoNet &locoNet, uint8_t userData, uint8_t options, uint16_t throttleId);
        bool processMessage(const lnMsg *LnPacket);
        void process100msActions(void);

        /**
         * Registers a callback for when the address for this throttle changes
         */
        void onAddressChange(std::function<void(LocoNetThrottle *, uint16_t, uint16_t)> callback) {
            addressChangeCallback = callback;
        }
        /**
         * Registers a callback for when the speed for this throttle changes
         */
        void onSpeedChange(std::function<void(LocoNetThrottle *, uint8_t)> callback) {
            speedChangeCallback = callback;
        }
        /**
         * Registers a callback for when the direction for this throttle changes
         */
        void onDirectionChange(std::function<void(LocoNetThrottle *, uint8_t)> callback) {
            directionChangeCallback = callback;
        }
        /**
         * Registers a callback for when a function for this throttle changes
         */
        void onFunctionChange(std::function<void(LocoNetThrottle *, uint8_t, bool)> callback) {
            functionChangeCallback = callback;
        }
        /**
         * Registers a callback for when this throttle changes slots
         */
        void onSlotStateChange(std::function<void(LocoNetThrottle *, uint8_t)> callback) {
            throttleSlotStateCallback = callback;
        }
        /**
         * Registers a callback for when a this throttle has an error
         */
        void onError(std::function<void(LocoNetThrottle *, TH_ERROR)> callback) {
            throttleErrorCallback = callback;
        }
        /**
         * Registers a callback for when this throttle changes status
         *                                                             Old Status  New Status
         */
        void onThrottleStateChange(std::function<void(LocoNetThrottle *, TH_STATE, TH_STATE)> callback) {
            throttleStateCallback = callback;
        }

        uint16_t getAddress(void);
        TH_ERROR setAddress(uint16_t Address);
        TH_ERROR resumeAddress(uint16_t Address, uint8_t LastSlot);
        TH_ERROR dispatchAddress(uint16_t Address);
        TH_ERROR acquireAddress(void);
        void releaseAddress(void);
        TH_ERROR freeAddress(uint16_t Address);

        uint8_t getSpeed(void);
        TH_ERROR setSpeed(uint8_t Speed);

        uint8_t getDirection(void);
        TH_ERROR setDirection(uint8_t Direction);

        uint8_t getFunction(uint8_t Function);
        TH_ERROR setFunction(uint8_t Function, uint8_t Value);
        TH_ERROR setDirFunc0to4Direct(uint8_t Value);
        TH_ERROR setFunc5to8Direct(uint8_t Value);

        TH_STATE getState(void);
        const char *getStateStr(TH_STATE State);
        const char *getErrorStr(TH_ERROR Error);
    private:
        LocoNet &_locoNet;
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

        void updateAddress(uint16_t Address, uint8_t ForceNotify);
        void updateSpeed(uint8_t Speed, uint8_t ForceNotify);
        void updateState(TH_STATE State, uint8_t ForceNotify);
        void updateStatus1(uint8_t Status, uint8_t ForceNotify);
        void updateDirectionAndFunctions(uint8_t DirFunc0to4, uint8_t ForceNotify);
        void updateFunctions5to8(uint8_t Func5to8, uint8_t ForceNotify);
        std::function<void(LocoNetThrottle *, uint16_t, uint16_t)> addressChangeCallback;
        std::function<void(LocoNetThrottle *, uint8_t)> speedChangeCallback;
        std::function<void(LocoNetThrottle *, uint8_t, bool)> functionChangeCallback;
        std::function<void(LocoNetThrottle *, uint8_t)> directionChangeCallback;
        std::function<void(LocoNetThrottle *, uint8_t)> throttleSlotStateCallback;
        std::function<void(LocoNetThrottle *, TH_STATE, TH_STATE)> throttleStateCallback;
        std::function<void(LocoNetThrottle *, TH_ERROR)> throttleErrorCallback;
};