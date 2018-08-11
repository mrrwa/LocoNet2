#ifndef LOCONETESP32_INCLUDED
#define LOCONETESP32_INCLUDED

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

#include "utility/LocoNet.h"

#define RX_BUFFER_SIZE	64
#define LN_ST_IDLE            0   // net is free for anyone to start transmission
#define LN_ST_CD_BACKOFF      1   // timer interrupt is counting backoff bits
#define LN_ST_TX_COLLISION    2   // just sending break after creating a collision
#define LN_ST_TX              3   // transmitting a packet
#define LN_ST_RX              4   // receiving bytes

#define LN_COLLISION_TICKS 15
#define LN_TX_RETRIES_MAX  25

extern "C" void USART_TX_vect(void) __attribute__ ((signal));

class LocoNetESP32Class: public LocoNetClass
{

    private:

        uint8_t txBuffer[ LN_BUF_SIZE];

        volatile uint16_t lnCompareTarget;
        LnRxStats* rxStats;
        volatile uint8_t txBytesRemaining;
        static volatile uint8_t rxBufferBytes[ LN_BUF_SIZE];
        volatile uint8_t rxHead = 0;
        volatile uint8_t rxTail = 0;
        volatile uint8_t* lnCurrentTxBytePtr;
        volatile uint8_t errCount;

        volatile lnMsg * volatile lnTxData;
        volatile uint8_t lnTxIndex;
        volatile uint8_t lnTxLength;

    public:
        void init();

        LN_STATUS sendLocoNetPacketTry(lnMsg *txData, unsigned char ucPrioDelay);
        static void IRAM_ATTR loconetStartBit();
        static void IRAM_ATTR loconetBitTimer();
        static bool CheckCollision();
        void TaskRun();

        static void locoNetESP32Task(void* pvParams)
        {
            while(true)
            {
                ((LocoNetESP32Class*)pvParams)->TaskRun();
            }
        }

};

#endif
