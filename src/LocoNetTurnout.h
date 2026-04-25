#pragma once

#include "LocoNet2.h"

/**
 * @brief A class for controlling a single turnout on the LocoNet bus.
 *
 */
class LocoNetTurnout {
    public:
        /**
         * @brief Construct a new LocoNetTurnout object
         *
         * @param bus A pointer to the LocoNetBus object
         */
        LocoNetTurnout(LocoNetBus* bus);

        /**
         * @brief Set the Address of the turnout
         *
         * @param address The address of the turnout (1-2048)
         */
        void setAddress(uint16_t address);

        /**
         * @brief Set the State of the turnout
         *
         * @param thrown True for thrown, false for closed
         */
        void setState(bool thrown);

    private:
        LocoNetBus* _bus;
        uint16_t _address;
        bool _thrown;
};
