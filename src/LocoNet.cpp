/****************************************************************************
 * 	Copyright (C) 2009..2013 Alex Shepherd
 *	Copyright (C) 2013 Damian Philipp
 *
 * 	Portions Copyright (C) Digitrax Inc.
 *	Portions Copyright (C) Uhlenbrock Elektronik GmbH
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

#include <string.h>

#include "LocoNet.h"

const char * LoconetStatusStrings[] = {
	"CD Backoff",
	"Prio Backoff",
	"Network Busy",
	"Done",
	"Collision",
	"Unknown Error",
	"Retry Error"
};

LocoNet::LocoNet() {
}

const char* LocoNet::getStatusStr(LN_STATUS Status) {
  if ((Status >= LN_CD_BACKOFF) && (Status <= LN_RETRY_ERROR)) {
    return LoconetStatusStrings[Status];
  }

  return "Invalid Status";
}

LN_STATUS LocoNet::send(lnMsg *pPacket, uint8_t ucPrioDelay) {
  LN_STATUS enReturn;
  bool ucWaitForEnterBackoff;

  /* First calculate the checksum as it may not have been done */
  uint8_t *packet = reinterpret_cast<uint8_t *>(pPacket);
  uint8_t packetLen = ((pPacket->sz.command & 0x60) == 0x60) ? pPacket->sz.mesg_size : ((pPacket->sz.command & 0x60) >> 4) + 2;
  uint8_t packetChecksum = 0xFF;
  for(uint8_t lnTxIndex = 0; lnTxIndex < packetLen - 1; lnTxIndex++) {
    packetChecksum ^= packet[lnTxIndex];
  }
  packet[packetLen - 1] = packetChecksum;

  for (uint8_t ucTry = 0; ucTry < LN_TX_RETRIES_MAX; ucTry++) {
    // wait previous traffic and than prio delay and than try tx
    // don't want to abort do/while loop before we did not see the backoff state once
    ucWaitForEnterBackoff = true;
    do {
      enReturn = sendLocoNetPacketTry(packet, packetLen, ucPrioDelay);

      if (enReturn == LN_DONE) { // success?
        return LN_DONE;
      }

      if (enReturn == LN_PRIO_BACKOFF) {
        // now entered backoff -> next state != LN_BACKOFF is worth incrementing the try counter
        ucWaitForEnterBackoff = false;
      }
    } while ((enReturn == LN_CD_BACKOFF) ||                             // waiting CD backoff
             (enReturn == LN_PRIO_BACKOFF) ||                           // waiting master+prio backoff
            ((enReturn == LN_NETWORK_BUSY) && ucWaitForEnterBackoff));  // or within any traffic unfinished
    // failed -> next try going to higher prio = smaller prio delay
    if (ucPrioDelay > LN_BACKOFF_MIN) {
      ucPrioDelay--;
    }
  }
  txStats.txErrors++;
  return LN_RETRY_ERROR;
}

LN_STATUS LocoNet::send(lnMsg *pPacket) {
  return send(pPacket, LN_BACKOFF_INITIAL);
}

LN_STATUS LocoNet::send( uint8_t OpCode, uint8_t Data1, uint8_t Data2 ) {
  lnMsg SendPacket ;

  SendPacket.data[ 0 ] = OpCode ;
  SendPacket.data[ 1 ] = Data1 ;
  SendPacket.data[ 2 ] = Data2 ;

  return send( &SendPacket ) ;
}

LN_STATUS LocoNet::send( uint8_t OpCode, uint8_t Data1, uint8_t Data2, uint8_t PrioDelay ) {
  lnMsg SendPacket ;

  SendPacket.data[ 0 ] = OpCode ;
  SendPacket.data[ 1 ] = Data1 ;
  SendPacket.data[ 2 ] = Data2 ;

  return send( &SendPacket, PrioDelay ) ;
}

LnRxStats* LocoNet::getRxStats(void) {
  return &rxBuffer.stats;
}

LnTxStats* LocoNet::getTxStats(void) {
  return &txStats;
}

LN_STATUS LocoNet::reportPower(uint8_t State) {
  lnMsg SendPacket ;

  if (State) {
    SendPacket.data[ 0 ] = OPC_GPON ;
  } else {
    SendPacket.data[ 0 ] = OPC_GPOFF ;
  }

  return send( &SendPacket ) ;
}

bool LocoNet::process() {
	return false;
}

bool LocoNet::process(uint8_t newByte) {
	bool isConsumed = false;
	lnMsg * rxPacket = rxBuffer.addByte( newByte);

	if (rxPacket) {
    if(callbacks.find(0xFF) != callbacks.end()) {
      for(auto cb : callbacks[0xFF]) {
        if(cb(rxPacket)) {
          isConsumed = true;
        }
      }
    }
    if(callbacks.find(rxPacket->sz.command) != callbacks.end()) {
      for(auto cb : callbacks[rxPacket->sz.command]) {
        if(cb(rxPacket)) {
          isConsumed = true;
        }
      }
    }
    if(!isConsumed) {
		  isConsumed = processSwitchSensorMessage(rxPacket);
    }
	}

	return isConsumed;
}

bool LocoNet::processSwitchSensorMessage( lnMsg *LnPacket ) {
  uint16_t Address ;
  uint8_t  Direction ;
  uint8_t  Output ;
  bool  isConsumed = true ;

  Address = (LnPacket->srq.sw1 | ( ( LnPacket->srq.sw2 & 0x0F ) << 7 )) ;
  if( LnPacket->sr.command != OPC_INPUT_REP ) {
    Address++;
  }

  switch( LnPacket->sr.command ) {
  case OPC_INPUT_REP:
    Address <<= 1 ;
    Address += ( LnPacket->ir.in2 & OPC_INPUT_REP_SW ) ? 2 : 1 ;

    if(notifySensor)
      notifySensor( Address, LnPacket->ir.in2 & OPC_INPUT_REP_HI ) ;
    break ;

  case OPC_GPON:
    if(notifyPower)
      notifyPower( 1 );
    break ;

  case OPC_GPOFF:
    if(notifyPower)
      notifyPower( 0 );
    break ;

  case OPC_SW_REQ:
    if(notifySwitchRequest)
      notifySwitchRequest( Address, LnPacket->srq.sw2 & OPC_SW_REQ_OUT, LnPacket->srq.sw2 & OPC_SW_REQ_DIR ) ;
    break ;

  case OPC_SW_REP:
    if(notifySwitchReport)
      notifySwitchReport( Address, LnPacket->srp.sn2 & OPC_SW_REP_HI, LnPacket->srp.sn2 & OPC_SW_REP_SW ) ;
    break ;

  case OPC_SW_STATE:
    Direction = LnPacket->srq.sw2 & OPC_SW_REQ_DIR ;
    Output = LnPacket->srq.sw2 & OPC_SW_REQ_OUT ;

    if(notifySwitchState)
      notifySwitchState( Address, Output, Direction ) ;
    break;

  case OPC_SW_ACK:
    break ;

  case OPC_LONG_ACK:
    if( LnPacket->lack.opcode == (OPC_SW_STATE & 0x7F ) ) {
      Direction = LnPacket->lack.ack1 & 0x01 ;
    } else {
      isConsumed = false;
    }
    break;

  default:
    isConsumed = false;
  }

  return isConsumed ;
}

LN_STATUS LocoNet::requestSwitch( uint16_t Address, uint8_t Output, uint8_t Direction ) {
  uint8_t AddrH = (--Address >> 7) & 0x0F ;
  uint8_t AddrL = Address & 0x7F ;

  if ( Output ) {
    AddrH |= OPC_SW_REQ_OUT ;
  }

  if ( Direction ) {
    AddrH |= OPC_SW_REQ_DIR ;
  }

  return send( OPC_SW_REQ, AddrL, AddrH ) ;
}

LN_STATUS LocoNet::reportSwitch( uint16_t Address ) {
  Address -= 1;
  return send( OPC_SW_STATE, (Address & 0x7F), ((Address >> 7) & 0x0F) ) ;
}

LN_STATUS LocoNet::reportSensor( uint16_t Address, uint8_t State ) {
	uint8_t AddrH = ( (--Address >> 8) & 0x0F ) | OPC_INPUT_REP_CB ;
	uint8_t AddrL = ( Address >> 1 ) & 0x7F ;
	if( Address % 2) {
		AddrH |= OPC_INPUT_REP_SW ;
  }

	if( State ) {
		AddrH |= OPC_INPUT_REP_HI  ;
  }

  return send( OPC_INPUT_REP, AddrL, AddrH ) ;
}

void LocoNet::onPacket(uint8_t OpCode, OpCodeCallbackFunction callback) {
  callbacks[OpCode].push_back(callback);
}
