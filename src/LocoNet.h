/****************************************************************************
 * 	Copyright (C) 2009 to 2013 Alex Shepherd
 * 	Copyright (C) 2013 Damian Philipp
 *
 * 	Portions Copyright (C) Digitrax Inc.
 * 	Portions Copyright (C) Uhlenbrock Elektronik GmbH
 *
 * 	This library is free software; you can redistribute it and/or
 * 	modify it under the terms of the GNU Lesser General Public
 * 	License as published by the Free Software Foundation; either
 * 	version 2.1 of the License, or (at your option) any later version.
 *
 * 	This library is distributed in the hope that it will be useful,
 * 	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * 	Lesser General Public License for more details.
 *
 * 	You should have received a copy of the GNU Lesser General Public
 * 	License along with this library; if not, write to the Free Software
 * 	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *****************************************************************************
 *
 * 	IMPORTANT:
 *
 * 	Some of the message formats used in this code are Copyright Digitrax, Inc.
 * 	and are used with permission as part of the MRRwA (previously EmbeddedLocoNet) project.
 *  That permission does not extend to uses in other software products. If you wish
 * 	to use this code, algorithm or these message formats outside of
 * 	MRRwA, please contact Digitrax Inc, for specific permission.
 *
 * 	Note: The sale any LocoNet device hardware (including bare PCB's) that
 * 	uses this or any other LocoNet software, requires testing and certification
 * 	by Digitrax Inc. and will be subject to a licensing agreement.
 *
 * 	Please contact Digitrax Inc. for details.
 *
 *****************************************************************************
 *
 * 	IMPORTANT:
 *
 * 	Some of the message formats used in this code are Copyright Uhlenbrock Elektronik GmbH
 * 	and are used with permission as part of the MRRwA (previously EmbeddedLocoNet) project.
 *  That permission does not extend to uses in other software products. If you wish
 * 	to use this code, algorithm or these message formats outside of
 * 	MRRwA, please contact Copyright Uhlenbrock Elektronik GmbH, for specific permission.
 *
 *****************************************************************************
 * 	DESCRIPTION
 * 	This module provides functions that manage the sending and receiving of LocoNet packets.
 *
 * 	As bytes are received from the LocoNet, they are stored in a circular
 * 	buffer and after a valid packet has been received it can be read out.
 *
 * 	When packets are sent successfully, they are also appended to the Receive
 * 	circular buffer so they can be handled like they had been received from
 * 	another device.
 *
 * 	Statistics are maintained for both the send and receiving of packets.
 *
 * 	Any invalid packets that are received are discarded and the stats are
 * 	updated approproately.
 *
 *****************************************************************************/

#pragma once

#include <map>
#include <etl/vector.h>
#include <vector>
#include <functional>

#include "ln_opc.h"
#include "LocoNetMessageBuffer.h"
#include "Bus.h"

#define DEBUG_OUTPUT_

#ifdef DEBUG_OUTPUT
#include <cstdio>
#if defined(ESP32) && ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_DEBUG
#include <esp32-hal-log.h>
#define DEBUG(format, ...) log_printf(ARDUHAL_LOG_FORMAT(D, format), ##__VA_ARGS__)
#define DEBUG_ISR(format, ...) ets_printf(ARDUHAL_LOG_FORMAT(D, format), ##__VA_ARGS__)
#else
#define DEBUG(...) { printf(__VA_ARGS__); printf("\n"); }
#define DEBUG_ISR(...) { printf(__VA_ARGS__); printf("\n"); }
#endif
#else
#define DEBUG(format, ...)
#define DEBUG_ISR(format, ...)
#endif

typedef enum
{
    LN_CD_BACKOFF = 0, LN_PRIO_BACKOFF, LN_NETWORK_BUSY, LN_DONE, LN_COLLISION, LN_UNKNOWN_ERROR, LN_RETRY_ERROR
} LN_STATUS;


using LocoNetBus = Bus<LnMsg, LN_STATUS, LN_STATUS::LN_DONE, 10>;

using LocoNetConsumer = Consumer<LnMsg, LN_STATUS>;

// CD Backoff starts after the Stop Bit (Bit 9) and has a minimum or 20 Bit Times
// but initially starts with an additional 20 Bit Times
constexpr uint8_t LN_CARRIER_TICKS      = 20; // carrier detect backoff - all devices have to wait this
constexpr uint8_t LN_MASTER_DELAY       = 6;  // non master devices have to wait this additionally
constexpr uint8_t LN_INITIAL_PRIO_DELAY = 20; // initial attempt adds priority delay
constexpr uint8_t LN_BACKOFF_MIN        = (LN_CARRIER_TICKS + LN_MASTER_DELAY);      // not going below this
constexpr uint8_t LN_BACKOFF_INITIAL    = (LN_BACKOFF_MIN + LN_INITIAL_PRIO_DELAY);  // for the first normal tx attempt
constexpr uint8_t LN_BACKOFF_MAX        = (LN_BACKOFF_INITIAL + 10);                 // lower priority is not supported
constexpr uint8_t LN_COLLISION_TICKS    = 15; //< after collision the bus will be low for this number of ticks.

//
// LNCV error codes
// Used by the LNCV callbacks to signal what kind of error has occurred.
//

// Error-codes for write-requests
#define LNCV_LACK_ERROR_GENERIC (0)
// Unsupported/non-existing CV
#define LNCV_LACK_ERROR_UNSUPPORTED (1)
// CV is read only
#define LNCV_LACK_ERROR_READONLY (2)
// Value out of range
#define LNCV_LACK_ERROR_OUTOFRANGE (3)
// Everything OK
#define LNCV_LACK_OK (127)

// the valid range for module addresses (CV0) as per the LNCV spec.
#define LNCV_MIN_MODULEADDR (0)
#define LNCV_MAX_MODULEADDR (65534)

#define LN_TX_RETRIES_MAX  25

constexpr uint8_t CALLBACK_FOR_ALL_OPCODES=0xFF;

inline uint8_t lnPacketSize(const LnMsg * msg) {
    return LOCONET_PACKET_SIZE(msg->sz.command, msg->sz.mesg_size);
}

#define ADDR(hi,lo)  (   ((lo) | (((hi) & 0x0F ) << 7))    )

#define MAX_BACKEND_CONSUMERS  10

class LocoNetPhy: public LocoNetConsumer {
    public:
        LocoNetPhy(LocoNetBus * bus);
        virtual bool begin();
        virtual void end();
        LN_STATUS send(LnMsg *txPacket);
        LN_STATUS send(LnMsg *txPacket, uint8_t PrioDelay);

        LnRxStats* getRxStats(void);
        LnTxStats* getTxStats(void);

        const char* getStatusStr(LN_STATUS status);
    
        LN_STATUS onMessage(const LnMsg& msg);

    protected:
        void consume(uint8_t newByte);

        virtual LN_STATUS sendLocoNetPacketTry(uint8_t *packetData, uint8_t packetLen, unsigned char ucPrioDelay) = 0;
        LocoNetMessageBuffer rxBuffer;
        LnTxStats txStats;

        LocoNetBus *bus;

};

LnMsg makeLongAck(uint8_t replyToOpc, uint8_t ack);

LnMsg makeMsg(uint8_t OpCode, uint8_t Data1, uint8_t Data2);

LnMsg makeSwRec(uint16_t address, bool output, bool thrown);

LN_STATUS requestSwitch(LocoNetBus *ln, uint16_t Address, uint8_t Output, uint8_t Direction);
LN_STATUS reportSwitch(LocoNetBus *ln, uint16_t Address);
LN_STATUS reportSensor(LocoNetBus *ln, uint16_t Address, uint8_t State);
LN_STATUS reportPower(LocoNetBus *ln, bool state);

class LocoNetDispatcher : public LocoNetConsumer {
    public:
        LocoNetDispatcher(LocoNetBus *ln);
        void begin() {}
        void end() {}

        LN_STATUS send(LnMsg *txPacket);
        //LN_STATUS send(LnMsg *TxPacket, uint8_t PrioDelay);
        LN_STATUS send(uint8_t opCode, uint8_t data1, uint8_t data2);
        //LN_STATUS send(uint8_t OpCode, uint8_t Data1, uint8_t Data2, uint8_t PrioDelay);
        
        LN_STATUS onMessage(const LnMsg& msg);

        void processPacket(const LnMsg *packet);

        void onPacket(uint8_t opCode, std::function<void(const LnMsg *)> callback);
        /**
         * Registers a callback for when a sensor changes state
         *                                     address   state
         */
        void onSensorChange(std::function<void(uint16_t, bool)> callback);
        /**
         * Registers a callback for when a switch is requested
         *                                     address   output direction
         */
        void onSwitchRequest(std::function<void(uint16_t, bool, bool)> callback);
        /**
         * Registers a callback for when a switch/sensor status is reported
         *                                     address   state switch/sensor
         */
        void onSwitchReport(std::function<void(uint16_t, bool, bool)> callback);
        /**
         * Registers a callback for when a switch is requested
         *                                     address   output direction
         */
        void onSwitchState(std::function<void(uint16_t, bool, bool)> callback);

        /**
         * Registers a callback for when power status changes
         *                                    on/off
         */
        void onPowerChange(std::function<void(bool)> callback);

        /**
         * Registers a callback for when a MultiSense device reports status
         *                                             id       index   AR/CB  Active
         * AR = Auto-Reversing (true)
         * CB = Circuit Breaker (false)
         */
        void onMultiSenseDeviceInfo(std::function<void(uint8_t, uint8_t, bool, bool)> callback);

        /**
         * Registers a callback for when a MultiSense Transponder event is triggered
         *                                              address    zone    locoaddr  presense
         */
        void onMultiSenseTransponder(std::function<void(uint16_t, uint8_t, uint16_t, bool)> callback);

    private:
        LocoNetBus * ln;
        bool processSwitchSensorMessage(LnMsg *lnPacket);
        std::map<uint8_t, std::vector<std::function<void(const LnMsg *)> > > callbacks;
};

using LocoNet = LocoNetDispatcher;

/************************************************************************************
 Call-back functions
 ************************************************************************************/

// LNCV notify Call-back functions

// Negative return codes will result in no message being sent.
// Where a value response is appropriate, a return value of LNCV_LACK_OK will trigger the
// response being sent.
// Other values greater than 0 will result in a LACK message being sent.
// When no value result is appropriate, LNCV_LACK_OK will be sent as a LACK.

/**
 * TODO: General LNCV documentation
 * Pick an ArtNr
 * Implement your code to the following behaviour...
 */
