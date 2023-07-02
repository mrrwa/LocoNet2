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

#ifdef ARDUINO_ARCH_RP2040

#include <LocoNetStreamRP2040.h>

LocoNetStreamRP2040::LocoNetStreamRP2040(SerialUART * serialPort, int8_t rxPin, int8_t txPin, LocoNetBus *bus) : LocoNetStream(bus)
{
	_serialPort = serialPort;
	_rxPin = rxPin;
	_txPin = txPin;
};

void LocoNetStreamRP2040::start(void)
{
	begin(_serialPort);
	_serialPort->setRX(_rxPin);
	_serialPort->setTX(_txPin);
	_serialPort->setFIFOSize(8);
	_serialPort->begin(LOCONET_BAUD);
	gpio_set_outover(_txPin, GPIO_OVERRIDE_INVERT);
	gpio_set_inover(_rxPin, GPIO_OVERRIDE_INVERT);
	
	_instance = this;
	attachInterrupt(_rxPin, isr, FALLING);
};

void LocoNetStreamRP2040::finish(void)
{
	detachInterrupt(_rxPin);
}

// The Interrupt Handler glue was from this forum post:
// https://forum.arduino.cc/t/use-attachinterrupt-with-a-class-method-function/301108/9

LocoNetStreamRP2040 * LocoNetStreamRP2040::_instance;

void LocoNetStreamRP2040::isr(void)
{
  _instance->handleLocoNetActivityInterrupt();
}

void LocoNetStreamRP2040::handleLocoNetActivityInterrupt(void)
{
	_LastLocoNetActivityMicros = micros();
}

bool LocoNetStreamRP2040::isBusy(void)
{ 
	if(!gpio_get(_rxPin))	// If the Rx Pin is LOW then the UART will be active so return true
		return true;
		 
	return (micros() - _LastLocoNetActivityMicros) < LocoNetRxByteMicros;
};

void LocoNetStreamRP2040::beforeSend(void){};
void LocoNetStreamRP2040::afterSend(void){};
void LocoNetStreamRP2040::sendBreak(void){};

#endif