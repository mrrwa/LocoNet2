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

#pragma once

#include <Arduino.h>
#include <Stream.h>

#include "LocoNet2.h"

class LocoNetStream: public LocoNetPhy {
	public:
		LocoNetStream(LocoNetBus *bus) : LocoNetPhy(bus), _state(LN_IDLE){};
		
		void begin(Stream * serialPort);
		void end();
		void process();


	protected:
		LN_STATUS sendLocoNetPacketTry(uint8_t *packetData, uint8_t packetLen, unsigned char ucPrioDelay);

		virtual bool isBusy(void);
		virtual void sendBreak(void);
		virtual void beforeSend(void);
		virtual void afterSend(void);
		
	private:
		void startCollisionTimer();
		bool hasCollisionTimerExpired();
		void startCDBackoffTimer();
		bool hasCDBackoffTimerExpired(uint8_t PrioDelay = 0);
		
		Stream * 	_serialPort;
		uint64_t 	_cdBackoffStart;
		uint64_t 	_cdBackoffTimeout;
		uint64_t 	_collisionTimeout;
		LN_STATUS	_state;
};


