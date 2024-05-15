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

#ifdef ARDUINO_ARCH_STM32

#include <LocoNetStreamSTM32.h>

LocoNetStreamSTM32::LocoNetStreamSTM32(HardwareSerial * serialPort, uint8_t rxPin, uint8_t txPin, LocoNetBus *bus, bool rxPinInvert, bool txPinInvert) : LocoNetStream(bus)
{
	_serialPort = serialPort;
	_rxPin = rxPin;
	_txPin = txPin;
	_rxPinInvert = rxPinInvert;
	_txPinInvert = txPinInvert;
};

void LocoNetStreamSTM32::start(void)
{
	begin(_serialPort);
	
	_serialPort->setRx(_rxPin);
	_serialPort->setTx(_txPin);
	
// Who knows if the Rx and Tx Pin Inversion logic will work.
	UART_HandleTypeDef * handlePtr = _serialPort->getHandle();
		
    if(_rxPinInvert)
 	{
 		handlePtr->AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_RXINVERT_INIT;
	    handlePtr->AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
	}
	
	if(_txPinInvert)
	{
		handlePtr->AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_TXINVERT_INIT;
		handlePtr->AdvancedInit.TxPinLevelInvert = UART_ADVFEATURE_TXINV_ENABLE;
	}

	_serialPort->begin(LOCONET_BAUD);
	
	_instance = this;
	attachInterrupt(_rxPin, isr, FALLING);
};

void LocoNetStreamSTM32::finish(void)
{
	detachInterrupt(_rxPin);
}

// The Interrupt Handler glue was from this forum post:
// https://forum.arduino.cc/t/use-attachinterrupt-with-a-class-method-function/301108/9

LocoNetStreamSTM32 * LocoNetStreamSTM32::_instance;

void LocoNetStreamSTM32::isr(void)
{
  _instance->handleLocoNetActivityInterrupt();
}

void LocoNetStreamSTM32::handleLocoNetActivityInterrupt(void)
{
	_LastLocoNetActivityMicros = micros();
}

bool LocoNetStreamSTM32::isBusy(void)
{ 
	if(!digitalRead(_rxPin))	// If the Rx Pin is LOW then the UART will be active so return true
		return true;
		 
	return (micros() - _LastLocoNetActivityMicros) < LocoNetRxByteMicros;
};

void LocoNetStreamSTM32::beforeSend(void)
{
};

void LocoNetStreamSTM32::afterSend(void)
{
};

void LocoNetStreamSTM32::sendBreak(void)
{
// to-do figure out how to do the BREAK
//
// get the HAL to do it if supported 
//
// One option
// Invert the Tx Line polarity
// delayMicroseconds(CollisionTimeoutIncrement);
// Revert the Tx Line polarity
};

#endif