#include "LocoNetTurnout.h"
#include "ln_opc.h"

/**
 * @brief Construct a new LocoNetTurnout:: LocoNetTurnout object
 *
 * @param bus
 */
LocoNetTurnout::LocoNetTurnout(LocoNetBus* bus) {
    _bus = bus;
    _address = 0;
    _thrown = false;
}

/**
 * @brief Set the Address of the turnout
 *
 * @param address The address of the turnout (1-2048)
 */
void LocoNetTurnout::setAddress(uint16_t address) {
    _address = address;
}

/**
 * @brief Set the State of the turnout
 *
 * @param thrown True for thrown, false for closed
 */
void LocoNetTurnout::setState(bool thrown) {
    _thrown = thrown;

    LnMsg msg;
    msg.srq.command = OPC_SW_REQ;
    msg.srq.sw1 = (_address - 1) & 0x7F;
    msg.srq.sw2 = (((_address - 1) >> 7) & 0x0F) | (_thrown ? OPC_SW_REQ_DIR : 0) | 0;

    _bus->broadcast( msg );
}
