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

#include "LocoNet.h"

void LocoNetSystemVariable::init(LocoNet *lnInstance, uint8_t mfgId, uint8_t devId, uint16_t productId, uint8_t swVersion) {
	this->lnInstance = lnInstance;
    lnInstance->onPacket(OPC_PEER_XFER, [this](lnMsg *rxPacket) {
        if(this->processMessage(rxPacket) != SV_NOT_CONSUMED) {
            return true;
        }
        return false;
    });

	DeferredProcessingRequired = 0;
    DeferredSrcAddr = 0;

    this->mfgId = mfgId ;
	this->devId = devId ;
	this->productId = productId ;
    this->swVersion = swVersion ;
}

uint8_t LocoNetSystemVariable::isSVStorageValid(uint16_t Offset) {
#ifdef E2END
  return (Offset >= SV_ADDR_EEPROM_SIZE ) && (Offset <= E2END + 2) ;
#else
  return (Offset >= SV_ADDR_EEPROM_SIZE ) && (Offset <= 0xFF + 2) ;
#endif
}

bool LocoNetSystemVariable::CheckAddressRange(uint16_t startAddress, uint8_t Count) {
  while (Count != 0) {
    if (!isSVStorageValid(startAddress)) {
       lnInstance->send(OPC_LONG_ACK,(OPC_PEER_XFER & 0x7F), 42); // report invalid SV address error
       return false;
    }
    startAddress++;
    Count--;
  }

  return true; // all valid
}

uint16_t LocoNetSystemVariable::writeSVNodeId(uint16_t newNodeId) {
    writeSVStorage(SV_ADDR_NODE_ID_H, newNodeId >> (uint8_t) 8);
    writeSVStorage(SV_ADDR_NODE_ID_L, newNodeId & (uint8_t) 0x00FF);

    return readSVNodeId();
}

uint16_t LocoNetSystemVariable::readSVNodeId(void) {
    return (readSVStorage(SV_ADDR_NODE_ID_H) << 8 ) | readSVStorage(SV_ADDR_NODE_ID_L);
}

typedef union {
  uint16_t                   w;
  struct { uint8_t lo,hi; } b;
} U16_t;

typedef union {
  struct {
    U16_t unDestinationId;
    U16_t unMfgIdDevIdOrSvAddress;
    U16_t unproductId;
    U16_t unSerialNumber;
  } stDecoded;
  uint8_t abPlain[8];
} SV_Addr_t;

void decodePeerData( peerXferMsg *pMsg, uint8_t *pOutData ) {
  uint8_t	Index ;
  uint8_t	Mask ;
  uint8_t	* pInData ;
  uint8_t	* pBits ;

  Mask = 0x01 ;
  pInData = &pMsg->d1 ;
  pBits = &pMsg->pxct1 ;

  for( Index = 0; Index < 8; Index++) {
	  pOutData[Index] = *pInData ;
    if( *pBits & Mask )
    	pOutData[Index] |= 0x80 ;

    if( Index == 3 ) {
		  Mask = 0x01 ;
		  pInData = &pMsg->d5 ;
		  pBits = &pMsg->pxct2 ;
    } else {
    	Mask <<= 1 ;
      pInData++;
    }
  }
}

void encodePeerData( peerXferMsg *pMsg, uint8_t *pInData ){
  uint8_t	Index ;
  uint8_t	Mask ;
  uint8_t	* pOutData ;
  uint8_t	* pBits ;

  Mask = 0x01 ;
  pOutData = &pMsg->d1 ;
  pBits = &pMsg->pxct1 ;

  for( Index = 0; Index < 8; Index++) {
    *pOutData = pInData[Index] & 0x7F;	// fixed SBor040102
    if( pInData[Index] & 0x80 )
    	*pBits |= Mask ;

    if( Index == 3 ) {
		  Mask = 0x01 ;
		  pOutData = &pMsg->d5 ;
		  pBits = &pMsg->pxct2 ;
    } else {
      Mask <<= 1 ;
			pOutData++;
    }
  }
}

SV_STATUS LocoNetSystemVariable::processMessage(lnMsg *LnPacket ) {
 SV_Addr_t unData ;

  if( ( LnPacket->sv.mesg_size != 0x10 ) ||
      ( LnPacket->sv.command != OPC_PEER_XFER ) ||
      ( LnPacket->sv.sv_type != 0x02 ) ||
      ( LnPacket->sv.sv_cmd & 0x40 ) ||
      ( ( LnPacket->sv.svx1 & 0xF0 ) != 0x10 ) ||
      ( ( LnPacket->sv.svx2 & 0xF0 ) != 0x10 ) )
    return SV_NOT_CONSUMED ;

  decodePeerData( &LnPacket->px, unData.abPlain ) ;

#ifdef DEBUG_SV
    Serial.print("LNSV Src: ");
    Serial.print(LnPacket->sv.src);
    Serial.print("  Dest: ");
    Serial.print(unData.stDecoded.unDestinationId.w);
    Serial.print("  CMD: ");
    Serial.println(LnPacket->sv.sv_cmd, HEX);
#endif
  if ((LnPacket->sv.sv_cmd != SV_DISCOVER) &&
      (LnPacket->sv.sv_cmd != SV_CHANGE_ADDRESS) &&
      (unData.stDecoded.unDestinationId.w != readSVNodeId())) {
#ifdef DEBUG_SV
    Serial.print("LNSV Dest Not Equal: ");
    Serial.println(readSVNodeId());
#endif
    return SV_NOT_CONSUMED;
  }

  switch( LnPacket->sv.sv_cmd ) {
    case SV_WRITE_SINGLE:
        if (!CheckAddressRange(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, 1)) return SV_ERROR;
        writeSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, unData.abPlain[4]);
        // fall through intended!
    case SV_READ_SINGLE:
        if (!CheckAddressRange(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, 1)) return SV_ERROR;
        unData.abPlain[4] = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w);
        break;

    case SV_WRITE_MASKED:
        if (!CheckAddressRange(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, 1)) return SV_ERROR;
        // new scope for temporary local variables only
        {
         unsigned char ucOld = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w) & (~unData.abPlain[5]);
         unsigned char ucNew = unData.abPlain[4] & unData.abPlain[5];
         writeSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, ucOld | ucNew);
        }
        unData.abPlain[4] = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w);
        break;

    case SV_WRITE_QUAD:
        if (!CheckAddressRange(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, 4)) return SV_ERROR;
        writeSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w+0, unData.abPlain[4]);
        writeSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w+1, unData.abPlain[5]);
        writeSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w+2, unData.abPlain[6]);
        writeSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w+3, unData.abPlain[7]);
        // fall through intended!
    case SV_READ_QUAD:
        if (!CheckAddressRange(unData.stDecoded.unMfgIdDevIdOrSvAddress.w, 4)) return SV_ERROR;
        unData.abPlain[4] = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w+0);
        unData.abPlain[5] = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w+1);
        unData.abPlain[6] = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w+2);
        unData.abPlain[7] = readSVStorage(unData.stDecoded.unMfgIdDevIdOrSvAddress.w+3);
        break;

    case SV_DISCOVER:
        DeferredSrcAddr = LnPacket->sv.src ;
        DeferredProcessingRequired = 1 ;
        return SV_DEFERRED_PROCESSING_NEEDED ;
        break;

    case SV_IDENTIFY:
        unData.stDecoded.unDestinationId.w            = readSVNodeId();
        unData.stDecoded.unMfgIdDevIdOrSvAddress.b.hi = devId;
        unData.stDecoded.unMfgIdDevIdOrSvAddress.b.lo = mfgId ;
        unData.stDecoded.unproductId.w                = productId;
        unData.stDecoded.unSerialNumber.b.lo          = readSVStorage(SV_ADDR_SERIAL_NUMBER_L);
        unData.stDecoded.unSerialNumber.b.hi          = readSVStorage(SV_ADDR_SERIAL_NUMBER_H);
        break;

    case SV_CHANGE_ADDRESS:
        if((mfgId != unData.stDecoded.unMfgIdDevIdOrSvAddress.b.lo) || (devId != unData.stDecoded.unMfgIdDevIdOrSvAddress.b.hi))
          return SV_NOT_CONSUMED; // not addressed
        if(productId != unData.stDecoded.unproductId.w)
          return SV_NOT_CONSUMED; // not addressed
        if(readSVStorage(SV_ADDR_SERIAL_NUMBER_L) != unData.stDecoded.unSerialNumber.b.lo)
          return SV_NOT_CONSUMED; // not addressed
        if(readSVStorage(SV_ADDR_SERIAL_NUMBER_H) != unData.stDecoded.unSerialNumber.b.hi)
          return SV_NOT_CONSUMED; // not addressed

        if (writeSVNodeId(unData.stDecoded.unDestinationId.w) != unData.stDecoded.unDestinationId.w)
        {
          lnInstance->send(OPC_LONG_ACK,(OPC_PEER_XFER & 0x7F),44);  // failed to change address in non-volatile memory (not implemented or failed to write)
          return SV_CONSUMED_OK ; // the LN reception was ok, we processed the message
        }
        break;

    case SV_RECONFIGURE:
        break;  // actual handling is done after sending out the reply

    default:
        lnInstance->send(OPC_LONG_ACK,(OPC_PEER_XFER & 0x7F),43); // not yet implemented
        return SV_ERROR;
  }

  encodePeerData( &LnPacket->px, unData.abPlain ); // recycling the received packet

  LnPacket->sv.sv_cmd |= 0x40;    // flag the message as reply

  LN_STATUS lnStatus = lnInstance->send(LnPacket, LN_BACKOFF_INITIAL);

#ifdef DEBUG_SV
  Serial.print("LNSV Send Response - Status: ");
  Serial.println(lnStatus);   // report status value from send attempt
#endif

  if (lnStatus != LN_DONE) {
    // failed to send the SV reply message.  Send will NOT be re-tried.
    lnInstance->send(OPC_LONG_ACK,(OPC_PEER_XFER & 0x7F),44);  // indicate failure to send the reply
  }

  if (LnPacket->sv.sv_cmd == (SV_RECONFIGURE | 0x40)) {
#ifdef ESP32
    //esp_restart();  // reset the esp32
#else
    wdt_enable(WDTO_15MS);  // prepare for reset
    while (1) {}            // stop and wait for watchdog to knock us out
#endif
  }

  return SV_CONSUMED_OK;
}

SV_STATUS LocoNetSystemVariable::doDeferredProcessing( void ) {
  if( DeferredProcessingRequired )
  {
    lnMsg msg ;
    SV_Addr_t unData ;

    msg.sv.command = OPC_PEER_XFER ;
    msg.sv.mesg_size = 0x10 ;
    msg.sv.src = DeferredSrcAddr ;
    msg.sv.sv_cmd = SV_DISCOVER | 0x40 ;
    msg.sv.sv_type = 0x02 ;
    msg.sv.svx1 = 0x10 ;
    msg.sv.svx2 = 0x10 ;

    unData.stDecoded.unDestinationId.w            = readSVNodeId();
    unData.stDecoded.unMfgIdDevIdOrSvAddress.b.lo = mfgId;
    unData.stDecoded.unMfgIdDevIdOrSvAddress.b.hi = devId;
    unData.stDecoded.unproductId.w                = productId;
    unData.stDecoded.unSerialNumber.b.lo          = readSVStorage(SV_ADDR_SERIAL_NUMBER_L);
    unData.stDecoded.unSerialNumber.b.hi          = readSVStorage(SV_ADDR_SERIAL_NUMBER_H);

    encodePeerData( &msg.px, unData.abPlain );

    if( lnInstance->send( &msg ) != LN_DONE )
      return SV_DEFERRED_PROCESSING_NEEDED ;

    DeferredProcessingRequired = 0 ;
  }

  return SV_CONSUMED_OK ;
}