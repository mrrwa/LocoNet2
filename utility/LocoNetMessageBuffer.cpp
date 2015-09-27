/****************************************************************************
 * Copyright (C) 2004 Alex Shepherd
 * 
 * Portions Copyright (C) Digitrax Inc.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 *****************************************************************************
 * 
 * IMPORTANT:
 * 
 * Some of the message formats used in this code are Copyright Digitrax, Inc.
 * and are used with permission as part of the EmbeddedLocoNet project. That
 * permission does not extend to uses in other software products. If you wish
 * to use this code, algorithm or these message formats outside of
 * EmbeddedLocoNet, please contact Digitrax Inc, for specific permission.
 * 
 * Note: The sale any LocoNet device hardware (including bare PCB's) that
 * uses this or any other LocoNet software, requires testing and certification
 * by Digitrax Inc. and will be subject to a licensing agreement.
 * 
 * Please contact Digitrax Inc. for details.
 * 
 *****************************************************************************
 * 
 * Title :   LocoNet Buffer Source Code file
 * Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 * Date:     13-Feb-2004
 * Software:  AVR-GCC
 * Target:    AtMega8
 * 
 * DESCRIPTION
 * This module provides functions that manage the receiving of
 * LocoNet packets.
 * 
 * 	As bytes are received from the LocoNet, they are stored in a circular
 * 	buffer and after a valid packet has been received it can be read out.
 * 
 * 	Statistics of packets and errors maintained.
 * 
 * 	Any invalid packets that are received are discarded and the stats are
 * 	updated approproately.
 * 
 *****************************************************************************/

#include <string.h>

#include "LocoNetMessageBuffer.h"

#define		LN_CHECKSUM_SEED        (uint8_t)0xFF

LocoNetMessageBufferClass::LocoNetMessageBufferClass()
{
  memset( buffer, 0, sizeof(buffer)) ;
  index = 0;
  expLen = 0;
  checkSum = LN_CHECKSUM_SEED;
	memset(&stats, 0, sizeof(LnRxStats));
}

lnMsg *LocoNetMessageBufferClass::getMsg(void)
{
	if( index < 2)
		return NULL;
		
	if(!expLen)
			// If it's a fixed length packet, compute the length from the OPC_Code, else get the length from the byte 1  
		expLen = ( ( buffer[0] & (uint8_t)0x60 ) == (uint8_t)0x60 ) ? buffer[1] : ( ( buffer[0] & (uint8_t)0x60 ) >> (uint8_t)4 ) + (uint8_t)2 ;
	
	if(expLen == index)
	{
		if(checkSum == buffer[index-1])
		{
			stats.rxPackets++;
			return (lnMsg*)buffer;
		}
		else
			stats.rxErrors++;
	}
	
	return NULL;
}

uint8_t LocoNetMessageBufferClass::getMsgSize( volatile lnMsg * Msg )
{
  return ( ( Msg->sz.command & (uint8_t)0x60 ) == (uint8_t)0x60 ) ? Msg->sz.mesg_size : ( ( Msg->sz.command & (uint8_t)0x60 ) >> (uint8_t)4 ) + 2 ;
}

lnMsg *LocoNetMessageBufferClass::addByte(uint8_t newByte )
{
	if(index < LN_BUF_SIZE)
	{
			// Reset the buffer to empty when a LocoNet OPC code is received
		if(newByte & 0x80)
		{
			index = 0;
			expLen = 0;
		  checkSum = LN_CHECKSUM_SEED;
		}
		
		buffer[ index++ ] = newByte ;
		checkSum ^= newByte;
	}
	
	return getMsg();
}