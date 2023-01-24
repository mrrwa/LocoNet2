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

#ifdef ARDUINO_ARCH_ESP32

#include <LocoNetStream.h>
#include "soc/uart_struct.h"
#include "esp32-hal.h"

// The following line is fron the ESP32 SDK file: Arduino15/packages/esp32/hardware/esp32/2.0.4/tools/sdk/esp32/include/hal/esp32/include/hal/uart_ll.h
#if defined(ARDUINO_ESP32_MCU_esp32)
  #define UART_LL_GET_HW(num) (((num) == 0) ? (&UART0) : (((num) == 1) ? (&UART1) : (&UART2)))
  
#elif defined(ARDUINO_ESP32_MCU_esp32c3)
  #define UART_LL_GET_HW(num) (((num) == 0) ? (&UART0) : (&UART1))
#endif

class LocoNetStreamESP32: public LocoNetStream {
	public:
		LocoNetStreamESP32(int esp32UartNumber, int8_t rxPin, int8_t txPin, bool invert, LocoNetBus *bus) : LocoNetStream(bus)
		{
			_uart_nr = esp32UartNumber;
			_rxPin = rxPin;
			_txPin = txPin;
			_invert = invert;
			_serialPort = new HardwareSerial(_uart_nr);
		};
	
		void start(void)
		{
			_serialPort->begin(LOCONET_BAUD, SERIAL_8N1, _rxPin, _txPin, _invert);
			
			begin(_serialPort);
		}

	    void end(bool fullyTerminate = true);
	
		bool isBusy(void)
		{
			uart_dev_t *hw = UART_LL_GET_HW(_uart_nr);		 	

		#if defined(ARDUINO_ESP32_MCU_esp32c3)    
		  	return hw->fsm_status.st_urx_out != 0;
			
		#elif defined(ARDUINO_ESP32_MCU_esp32)  
			return hw->status.st_urx_out != 0;
		#else
		#error "Unsupported ESP32 Archutecture"
		#endif
		};

		void beforeSend(void)
		{
			_tempRxFifoThreshold = updateRxFifoFullThreshold(1);
		};
		
		void afterSend(void)
		{
			updateRxFifoFullThreshold(_tempRxFifoThreshold);
		};
		
		void sendBreak(void)
		{
			_serialPort->updateBaudRate(LOCONET_BREAK_BAUD);
			_serialPort->write((uint8_t) 0);
			_serialPort->flush();
			_serialPort->updateBaudRate(LOCONET_BAUD);
		};
		
	private:
		uint32_t			_uart_nr;
		HardwareSerial * 	_serialPort;
		uint32_t 			_tempRxFifoThreshold;
		int8_t				_rxPin;
		int8_t				_txPin;
		bool				_invert;

		uint32_t updateRxFifoFullThreshold(uint32_t newThreshold)
		{
		  uart_dev_t *hw = UART_LL_GET_HW(_uart_nr);

		  uint32_t oldThreshold = hw->conf1.rxfifo_full_thrhd;
		  hw->conf1.rxfifo_full_thrhd = newThreshold;
	  
		  return oldThreshold;
		};


};
#endif	// ARDUINO_ARCH_ESP32
