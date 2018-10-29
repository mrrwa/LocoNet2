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
#include <string.h>
#include "LocoNet.h"

#define FC_FLAG_DCS100_COMPATIBLE_SPEED	0x01
#define FC_FLAG_MINUTE_ROLLOVER_SYNC	0x02
#define FC_FLAG_NOTIFY_FRAC_MINS_TICK	0x04
#define FC_FRAC_MIN_BASE   				0x3FFF
#define FC_FRAC_RESET_HIGH	 			0x78
#define FC_FRAC_RESET_LOW 	 			0x6D
#define FC_TIMER_TICKS         			65        // 65ms ticks
#define FC_TIMER_TICKS_REQ	  			250        // 250ms waiting for Response to FC Req

void LocoNetFastClock::init(LocoNet * lnInstance, uint8_t DCS100CompatibleSpeed, uint8_t CorrectDCS100Clock, uint8_t NotifyFracMin) {
    this->lnInstance = lnInstance;
	  lnInstance->onPacket(OPC_WR_SL_DATA, std::bind(&LocoNetFastClock::processMessage, this, std::placeholders::_1));
    lnInstance->onPacket(OPC_SL_RD_DATA, std::bind(&LocoNetFastClock::processMessage, this, std::placeholders::_1));
    lnInstance->onPacket(FC_SLOT, std::bind(&LocoNetFastClock::processMessage, this, std::placeholders::_1));

    fcState = FC_ST_IDLE ;

    fcFlags = 0;
    if(DCS100CompatibleSpeed) {
        fcFlags |= FC_FLAG_DCS100_COMPATIBLE_SPEED ;
    }

    if(CorrectDCS100Clock) {
        fcFlags |= FC_FLAG_MINUTE_ROLLOVER_SYNC ;
    }

    if(NotifyFracMin) {
        fcFlags |= FC_FLAG_NOTIFY_FRAC_MINS_TICK ;
    }
}

void LocoNetFastClock::poll() {
  lnInstance->send( OPC_RQ_SL_DATA, FC_SLOT, 0 ) ;
}

void LocoNetFastClock::doNotify( uint8_t Sync ) {
  if(notifyFastClock)
	  notifyFastClock(fcSlotData.clk_rate, fcSlotData.days,
	    (fcSlotData.hours_24 >= (128-24)) ? fcSlotData.hours_24 - (128-24) : fcSlotData.hours_24 % 24 ,
	    fcSlotData.mins_60 - (127-60 ), Sync ) ;
}

bool LocoNetFastClock::processMessage( lnMsg *LnPacket ) {
  if( ( LnPacket->fc.slot == FC_SLOT ) && ( ( LnPacket->fc.command == OPC_WR_SL_DATA ) || ( LnPacket->fc.command == OPC_SL_RD_DATA ) ) ) {
    if( LnPacket->fc.clk_cntrl & 0x40 ) {
      if( fcState >= FC_ST_REQ_TIME ) {
		    memcpy( &fcSlotData, &LnPacket->fc, sizeof( fastClockMsg ) ) ;
        doNotify( 1 ) ;
        if( fcFlags & FC_FLAG_NOTIFY_FRAC_MINS_TICK )
          notifyFastClockFracMins( FC_FRAC_MIN_BASE - ( ( fcSlotData.frac_minsh << 7 ) + fcSlotData.frac_minsl ) );
        fcState = FC_ST_READY ;
      }
    } else {
      fcState = FC_ST_DISABLED ;
    }
    return true;
  }
  return false;
}

void LocoNetFastClock::process66msActions() {
		// If we are all initialised and ready then increment accumulators
  if( fcState == FC_ST_READY ) {
    fcSlotData.frac_minsl +=  fcSlotData.clk_rate ;
    if( fcSlotData.frac_minsl & 0x80 ) {
      fcSlotData.frac_minsl &= ~0x80 ;

      fcSlotData.frac_minsh++ ;
      if( fcSlotData.frac_minsh & 0x80 ) {
					// For the next cycle prime the fraction of a minute accumulators
        fcSlotData.frac_minsl = FC_FRAC_RESET_LOW ;

					// If we are in FC_FLAG_DCS100_COMPATIBLE_SPEED mode we need to run faster
					// by reducong the FRAC_MINS duration count by 128
        fcSlotData.frac_minsh = FC_FRAC_RESET_HIGH + (fcFlags & FC_FLAG_DCS100_COMPATIBLE_SPEED) ;

        fcSlotData.mins_60++;
        if( fcSlotData.mins_60 >= 0x7F ) {
          fcSlotData.mins_60 = 127 - 60 ;
          fcSlotData.hours_24++ ;
          if( fcSlotData.hours_24 & 0x80 ) {
            fcSlotData.hours_24 = 128 - 24 ;
            fcSlotData.days++;
          }
        }

        // We either send a message out onto the LocoNet to change the time,
        // which we will also see and act on or just notify our user
        // function that our internal time has changed.
        if( fcFlags & FC_FLAG_MINUTE_ROLLOVER_SYNC ) {
          fcSlotData.command = OPC_WR_SL_DATA ;
          lnInstance->send((lnMsg*)&fcSlotData) ;
        } else {
    			doNotify( 0 ) ;
        }
      }
    }
    if( notifyFastClockFracMins && (fcFlags & FC_FLAG_NOTIFY_FRAC_MINS_TICK ))
      notifyFastClockFracMins( FC_FRAC_MIN_BASE - ( ( fcSlotData.frac_minsh << 7 ) + fcSlotData.frac_minsl ) ) ;
  }

  if( fcState == FC_ST_IDLE ) {
    lnInstance->send( OPC_RQ_SL_DATA, FC_SLOT, 0 ) ;
    fcState = FC_ST_REQ_TIME ;
  }
}