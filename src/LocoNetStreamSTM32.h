/****************************************************************************
 * 	Copyright (C) 2023 Alex Shepherd
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

#ifdef ARDUINO_ARCH_STM32

#include <LocoNetStream.h>

class LocoNetStreamSTM32: public LocoNetStream {
public:
  LocoNetStreamSTM32(HardwareSerial * serialPort, uint8_t rxPin, uint8_t txPin, LocoNetBus *bus, bool rxPinInvert=false, bool txPinInvert=false);
  void start(void);
  void finish(void);

  bool isBusy(void);
  void beforeSend(void);
  void afterSend(void);
  void sendBreak(void);
  
  static void isr (void);
  void handleLocoNetActivityInterrupt(void);

private:
  HardwareSerial * 		_serialPort;
  uint8_t			_rxPin;
  bool				_rxPinInvert;
  uint8_t			_txPin;
  bool				_txPinInvert;
  
  static LocoNetStreamSTM32 * _instance;
  uint64_t volatile _LastLocoNetActivityMicros;
};
#endif