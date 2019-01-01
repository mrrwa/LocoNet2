#include "LocoNetESP32UART.h"

constexpr UBaseType_t LocoNetRXTXThreadPriority = 2;
constexpr uint32_t LocoNetRXTXThreadStackSize = 1600;

// number of microseconds for one bit
constexpr uint8_t LocoNetTickTime = 60;

// number of microseconds to remain in a collision state
constexpr uint32_t CollisionTimeoutIncrement = 15 * LocoNetTickTime;

// number of microseconds to remain in a CD BACKOFF state
constexpr uint32_t CDBackoffTimeoutIncrement = LocoNetTickTime * LN_CARRIER_TICKS;

LocoNetESP32Uart::LocoNetESP32Uart(uint8_t rxPin, uint8_t txPin, uint8_t uartNum, bool inverted) :
	LocoNet(), _rxPin(rxPin), _txPin(txPin), _inverted(inverted), _state(IDLE) {
	_uart = uartBegin(uartNum, 16667, SERIAL_8N1, _rxPin, _txPin, 256, _inverted);
	_rxtxTask = nullptr;
	_txQueue = xQueueCreate(256, sizeof(uint8_t));
	if(_txQueue == NULL) {
		printf("LocoNet ERROR: Failed to create TX queue!\n");
	}
}

void LocoNetESP32Uart::begin() {
	DEBUG("Starting LocoNet RX/TX Task");
	if(xTaskCreate(LocoNetESP32Uart::taskEntryPoint, "LocoNet RX/TX Task", LocoNetRXTXThreadStackSize, (void *)this, LocoNetRXTXThreadPriority, &_rxtxTask) != pdPASS) {
		printf("LocoNet ERROR: Failed to start LocoNet RX/TX task!\n");
	}
}

void LocoNetESP32Uart::end() {
	if(_rxtxTask) {
		DEBUG("Suspending LocoNet RX/TX Task");
		vTaskSuspend(_rxtxTask);
		DEBUG("Deleting LocoNet RX/TX Task");
		vTaskDelete(_rxtxTask);
		_rxtxTask = nullptr;
	}
}

void LocoNetESP32Uart::startCollisionTimer() {
	DEBUG("LocoNet Collision!!!");
	uartFlush(_uart);
	_state = TX_COLLISION;
	txStats.collisions++;
	_collisionTimeout = (uint64_t)esp_timer_get_time() + CollisionTimeoutIncrement;
}

bool LocoNetESP32Uart::checkCollisionTimer() {
	return _collisionTimeout <= (uint64_t)esp_timer_get_time();
}

void LocoNetESP32Uart::startCDBackoffTimer() {
	_state = CD_BACKOFF;
	_cdBackoffStart = (uint64_t)esp_timer_get_time();
	_cdBackoffTimeout = _cdBackoffStart + CDBackoffTimeoutIncrement;
}

bool LocoNetESP32Uart::checkCDBackoffTimer() {
	return _cdBackoffTimeout <= (uint64_t)esp_timer_get_time();
}

void LocoNetESP32Uart::rxtxTask() {
	while(true) {
		// process incoming first
		if(uartAvailable(_uart)) {
			// start RX to consume available data
			_state = RX;
			while(uartAvailable(_uart)) {
				consume(uartRead(_uart));
			}
			// successful RX, switch to CD_BACKOFF
			startCDBackoffTimer();
		} else if(_state == CD_BACKOFF && checkCDBackoffTimer()) {
			_state = IDLE;
		} else if(_state == IDLE) {
			if(_txQueue && uxQueueMessagesWaiting(_txQueue) > 0) {
				// last chance check for TX_COLLISION before starting TX
				if(digitalRead(_rxPin) == !_inverted ? LOW : HIGH) {
					startCollisionTimer();
				} else {
					// no collision, start TX
					_state = TX;
					while(uxQueueMessagesWaiting(_txQueue) > 0 && _state == TX) {
						uint8_t out;
						if(xQueueAltReceive(_txQueue, &out, (portTickType)1)) {
							// collision check between each byte
							if(digitalRead(_rxPin) == !_inverted ? LOW : HIGH) {
								startCollisionTimer();
							} else {
								uartWrite(_uart, out);
							}
						}
					}
					if(_state == TX) {
						// TX done, switch to CD_BACKOFF
						startCDBackoffTimer();
					} else {
						// discard TX queue as we had collision
						xQueueReset(_txQueue);
					}
				}
			}
		} else if(_state == TX_COLLISION && checkCollisionTimer()) {
			digitalWrite(_txPin, !_inverted ? LOW : HIGH);
			startCDBackoffTimer();
		} else {
			digitalWrite(_txPin, _inverted ? LOW : HIGH);
		}
		vPortYield();
	}
}

LN_STATUS LocoNetESP32Uart::sendLocoNetPacketTry(uint8_t *packetData, uint8_t packetLen, unsigned char ucPrioDelay)
{
	if (_state == CD_BACKOFF) {
		if(micros() < _cdBackoffStart + (LocoNetTickTime * ucPrioDelay)) {
			_state = IDLE;
		} else if(!checkCDBackoffTimer()) {
			return LN_CD_BACKOFF;
		} else {
			return LN_PRIO_BACKOFF;
		}
	} else if(_state != IDLE) {
		return LN_NETWORK_BUSY;
	}
	if(_txQueue) {
		for(uint8_t index = 0; index < packetLen && (_state == IDLE || _state == TX); index++) {
			while(xQueueAltSendToBack(_txQueue, &packetData[index], (portTickType)5) != pdPASS) {
				vPortYield();
			}
		}
		// wait for TX to complete
		while(_state == IDLE || _state == TX) {
			vPortYield();
		}
		if(_state == IDLE || _state == CD_BACKOFF) {
			return LN_DONE;
		} else if(_state == TX_COLLISION) {
			return LN_COLLISION;
		}
	}
	return LN_UNKNOWN_ERROR;
}