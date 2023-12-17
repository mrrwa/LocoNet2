/****************************************************************************
 * 	Copyright (C) 2015 Alex Shepherd
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

#ifdef ARDUINO_ARCH_ESP32

#pragma once

#include "LocoNet2.h"
#include <Arduino.h>
#include <esp32-hal-timer.h>


class LocoNetESP32: public LocoNetPhy
{
    public:
        LocoNetESP32(LocoNetBus *bus, uint8_t rxPin=16, uint8_t txPin=15, uint8_t timerId=0);
        virtual bool begin();
        virtual void end();

        LN_STATUS sendLocoNetPacketTry(uint8_t *packetData, uint8_t packetLen, unsigned char ucPrioDelay);
    private:
        typedef enum {
            LN_ST_IDLE = 0,     // net is free for anyone to start transmission
            LN_ST_CD_BACKOFF,   // timer interrupt is counting backoff bits
            LN_ST_TX_COLLISION, // just sending break after creating a collision
            LN_ST_TX,           // transmitting a packet
            LN_ST_RX            // receiving bytes
        } LN_TX_RX_STATUS;

        QueueHandle_t _rxQueue;
        QueueHandle_t _txQueue;
        LN_TX_RX_STATUS _state;
        portMUX_TYPE _timerMux = portMUX_INITIALIZER_UNLOCKED;
        uint8_t _lnCurrentTxByte;
        uint8_t _lnCurrentRxByte;
        uint8_t _currentBit;
        TaskHandle_t _rxByteTask;
        hw_timer_t * _lnTimer;

        const uint8_t _rxPin;
        const uint8_t _txPin;
        const uint8_t _timerId;
        enum {
            VAL_TX_LOW=HIGH,
            VAL_TX_HIGH=LOW
        };
        enum {
            VAL_RX_LOW=LOW,
            VAL_RX_HIGH=HIGH
        };

        bool _isrAttached = false;
        void IRAM_ATTR enableStartBitISR(bool en=true) ;
        void IRAM_ATTR loconetStartBit();
        void IRAM_ATTR loconetBitTimer();

        bool IRAM_ATTR checkCollision();
        typedef enum {NO_LOCK, LOCK, LOCK_FROM_ISR} Lock;
        void IRAM_ATTR changeState(LN_TX_RX_STATUS newStat, Lock ll=LOCK, uint8_t bit=0);

        void rxByteTaskFunc();

        static void rxByteProc(void* pvParams) {
            static_cast<LocoNetESP32*>(pvParams)->rxByteTaskFunc();
        }

        friend void locoNetTimerCallback();
        friend void locoNetStartBitCallback();

};
#endif