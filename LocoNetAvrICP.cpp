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

#include "LocoNetAvrICP.h"

void LocoNetAvrIcpClass::init(uint8_t txPin)
{
}

LN_STATUS LocoNetAvrIcpClass::sendLocoNetPacketTry(lnMsg *txData, unsigned char ucPrioDelay)
{
	txData = txData;						// Keep the Compilar happy
	ucPrioDelay = ucPrioDelay;

	return LN_DONE;
}

void LocoNetAvrIcpClass::debug(int32_t Data)
{
	Serial.print(Data);
}	

void LocoNetAvrIcpClass::debug(const char* Data) 
{
	Serial.print(Data);
}

void LocoNetAvrIcpClass::debug(uint32_t Data, uint8_t Base)
{
	Serial.print(Data, Base);
}