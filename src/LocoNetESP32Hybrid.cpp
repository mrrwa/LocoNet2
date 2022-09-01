#include "LocoNetESP32Hybrid.h"
#include <esp_task_wdt.h>


constexpr UBaseType_t LocoNetRXTXThreadPriority = 1;
constexpr uint32_t LocoNetRXTXThreadStackSize = 2048;

// number of microseconds for one bit
constexpr uint8_t LocoNetTickTime = 60;

// number of microseconds to remain in a collision state
constexpr uint32_t CollisionTimeoutIncrement = 15 * LocoNetTickTime;

// number of microseconds to remain in a CD BACKOFF state
constexpr uint32_t CDBackoffTimeoutIncrement = LocoNetTickTime * LN_CARRIER_TICKS;

#define LOCONET_TX_LOCK()    do {} while (xSemaphoreTake(_txQueuelock, portMAX_DELAY) != pdPASS)
#define LOCONET_TX_UNLOCK()  xSemaphoreGive(_txQueuelock)



extern "C" void uartDetachRx(uart_t* uart);
extern "C" void uartDetachTx(uart_t* uart);
extern "C" void uartAttachRx(uart_t* uart, uint8_t rxPin, bool inverted);
extern "C" void uartAttachTx(uart_t* uart, uint8_t txPin, bool inverted);

#define INV_VAL(V) ((V)==HIGH ? LOW : HIGH)
#define TX_LOW_VAL  (INV_VAL(TX_HIGH_VAL))
#define RX_LOW_VAL  (INV_VAL(RX_HIGH_VAL))
#define TX_IDLE_VAL  TX_HIGH_VAL

static LocoNetESP32Hybrid *_inst = nullptr;

void IRAM_ATTR txTimerCb() {
    _inst->txBitTimerFunc();
}

void taskEntryPoint(void *param) {
	static_cast<LocoNetESP32Hybrid *>(param)->rxtxTask();
}


LocoNetESP32Hybrid::LocoNetESP32Hybrid(LocoNetBus *bus, uint8_t rxPin, uint8_t txPin, uint8_t uartNum, 
		bool invertedRx, bool invertedTx, const int timerId, const BaseType_t preferedCore
		) :
	LocoNetPhy(bus), _rxPin(rxPin), _txPin(txPin), _invertedRx(invertedRx), _invertedTx(invertedTx), 
	_preferedCore(preferedCore), TX_HIGH_VAL(invertedTx?LOW:HIGH), RX_HIGH_VAL(invertedRx?LOW:HIGH), _state(IDLE), _timerId(timerId)
{
	_inst = this;

	DEBUG("Initializing UART%d with RX:%d(%c), TX:%d(%c), timer %d", uartNum, _rxPin, _invertedRx?'I':'n', _txPin, _invertedTx?'I':'n', _timerId);
	_uart = uartBegin(uartNum, 16667, SERIAL_8N1, _rxPin, -1, 256, 32, _invertedRx, 192);
	/*if(_invertedRx) {		
		uartDetachRx(_uart);
		uartAttachRx(_uart, _rxPin, true);
	}*/
	
	/*if(_invertedTx) {
		uartDetachTx(_uart);
		uartAttachTx(_uart, _txPin, true);
	}*/

	_rxtxTask = nullptr;
	// note: this needs to be done after uartBegin which will set the pin mode to INPUT only.
	/*
	if(enablePullup) {
		pinMode(_rxPin, invertedRx ? INPUT_PULLDOWN : INPUT_PULLUP);
	}
	*/

	pinMode(_txPin, OUTPUT);
	digitalWrite(_txPin, TX_IDLE_VAL); // release bus
	
}

bool LocoNetESP32Hybrid::begin() {
	DEBUG("Creating LocoNet TX Queue");
	_txQueue = xQueueCreate(256, sizeof(uint8_t));
	if(_txQueue == NULL) {
		printf("LocoNet ERROR: Failed to create TX queue!\n");
		return false;
	}
	DEBUG("Creating LocoNet TX Lock");
	_txQueuelock = xSemaphoreCreateMutex();
	if(_txQueuelock == NULL) {
		printf("LocoNet ERROR: Failed to create TX Lock!\n");
		return false;
	}
	DEBUG("Starting LocoNet RX/TX Task");
	if(xTaskCreatePinnedToCore(taskEntryPoint, "LocoNet RX/TX", LocoNetRXTXThreadStackSize,
		(void *)this, LocoNetRXTXThreadPriority, &_rxtxTask, _preferedCore) != pdPASS) {
		printf("LocoNet ERROR: Failed to start LocoNet RX/TX task!\n");
		return false;
	}

	_lnTimer = timerBegin(_timerId, 480, true);
	if(_lnTimer==nullptr) {
		printf("LocoNet ERROR: Could not create timer!\n");
		return false;
	}
    timerAttachInterrupt(_lnTimer, &txTimerCb, true);
    timerAlarmWrite(_lnTimer, 10, true);
	//timerAlarmWrite(_lnTimer, 10000, true);
	//timerAlarmEnable(_lnTimer);
	timerAlarmEnable(_lnTimer);
	timerStop(_lnTimer);

	return true;
}

void LocoNetESP32Hybrid::end() {
	if(_rxtxTask) {
		DEBUG("Suspending LocoNet RX/TX Task");
		vTaskSuspend(_rxtxTask);
		DEBUG("Deleting LocoNet RX/TX Task");
		vTaskDelete(_rxtxTask);
	}
	_rxtxTask = nullptr;
	if(_txQueue) {
		DEBUG("Deleting LocoNet TX Queue");
		vQueueDelete(_txQueue);
	}
	_txQueue = nullptr;
	if(_txQueuelock) {
		DEBUG("Deleting LocoNet TX Queue Lock");
		vSemaphoreDelete(_txQueuelock);
	}
	_txQueuelock = nullptr;
	if(_lnTimer) timerEnd(_lnTimer);
	_lnTimer = nullptr;
}

void LocoNetESP32Hybrid::startCollisionTimer() {
	DEBUG("LocoNet Collision!!!");
	uartFlush(_uart);
	_state = TX_COLLISION;
	txStats.collisions++;
	_collisionTimeout = (uint64_t)esp_timer_get_time() + CollisionTimeoutIncrement;
}

/**
 * @return false is collision timer is still ticking
 */
bool LocoNetESP32Hybrid::collisionTimerElapsed() {
	return _collisionTimeout <= (uint64_t)esp_timer_get_time();
}

void LocoNetESP32Hybrid::startCDBackoffTimer() {
	_state = CD_BACKOFF;
	_cdBackoffStart = (uint64_t)esp_timer_get_time();
	_cdBackoffTimeout = _cdBackoffStart + CDBackoffTimeoutIncrement;
}

bool LocoNetESP32Hybrid::cdBackoffTimerElapsed() {
	return _cdBackoffTimeout <= (uint64_t)esp_timer_get_time();
}

void LocoNetESP32Hybrid::rxtxTask() {
	// add this thread to the WDT
	//esp_task_wdt_add(NULL);

	while(true) {
		//esp_task_wdt_reset();
		// process incoming first
		
		if(uartAvailable(_uart)) {
			DEBUG("RX Begin");
			// start RX to consume available data
			_state = RX;
			while(uartAvailable(_uart)) {
				//esp_task_wdt_reset();
				consume(uartRead(_uart));
			}
			DEBUG("RX End");
			// successful RX, switch to CD_BACKOFF
			startCDBackoffTimer();
		} else if(_state == CD_BACKOFF && cdBackoffTimerElapsed()) {
			DEBUG("Switching to IDLE after backoff");
			_state = IDLE;
		} else if(_state == IDLE) {
			LOCONET_TX_LOCK();
			if(_txQueue && uxQueueMessagesWaiting(_txQueue) > 0) {
				DEBUG("TX Begin");
				// last chance check for TX_COLLISION before starting TX
				// st_urx_out contains the status of the UART RX state machine,
				// any value other than zero indicates it is active.
// AJS				if(uartRxActive(_uart) || digitalRead(_rxPin) == RX_LOW_VAL) {
				if(digitalRead(_rxPin) == RX_LOW_VAL) {
					DEBUG("uartActive: %d / digitalRead: %d",uartRxActive(_uart)?1:0,  digitalRead(_rxPin) == RX_LOW_VAL?1:0);
					startCollisionTimer();
				} else  {
					// no collision, start TX
					_state = TX;
					while(uxQueueMessagesWaiting(_txQueue) > 0 && _state == TX) {
						uint8_t out;
						uint32_t t0=0;
						if(xQueueReceive(_txQueue, &out, (portTickType)1)) {
							DEBUG("sending %02x -> %04x", out, txByte);
							txByte = 1<<9 | out<<1 | 0; 
							txBit = 0;
							//timerAlarmEnable(_lnTimer);
							
							timerRestart(_lnTimer);

							// wait for echo byte before sending next byte
							uint32_t t1=micros();
							while(!uartAvailable(_uart)) {
								esp_task_wdt_reset();
								//delay(1);
							}
							t0 += (micros()-t1);
							DEBUG("Took %d uS", t0);
							// check echoed byte for collision
							uint8_t tt = uartRead(_uart);
							if(tt != out) {
								DEBUG("Got %02x, expected %02x", tt, out);
								startCollisionTimer();
								digitalWrite(_txPin, TX_LOW_VAL);
							}
						}
						//esp_task_wdt_reset();
					}
					if(_state == TX) {
						// TX done, switch to CD_BACKOFF
						startCDBackoffTimer();
						DEBUG("TX complete");
					} else {
						// discard TX queue as we had collision
						xQueueReset(_txQueue);
						DEBUG("TX queue reset");
					}
				}
				DEBUG("TX End");
			}
			LOCONET_TX_UNLOCK();
		} else if(_state == TX_COLLISION && collisionTimerElapsed()) {
			digitalWrite(_txPin, TX_IDLE_VAL);
			startCDBackoffTimer();
			DEBUG("TX COLLISION TIMER elapsed");
		} else {
			digitalWrite(_txPin, TX_IDLE_VAL);
		}
		//esp_task_wdt_reset();
		//delay(1);
		yield();
	}
}

void LocoNetESP32Hybrid::txBitTimerFunc() {
	//timerAlarmWrite(_lnTimer, 10, true);
	bool bit = txByte & (1<<txBit);
	//if (txBit>9) return;
	//DEBUG_ISR("%d=%d", txBit, bit);
	digitalWrite( _txPin, bit ? TX_HIGH_VAL : TX_LOW_VAL );
	if(txBit==9) {
		timerStop(_lnTimer);
		//DEBUG_ISR("disabling %d", 1);
		//timerAlarmDisable(_lnTimer);
	}
	txBit++;
}

LN_STATUS LocoNetESP32Hybrid::sendLocoNetPacketTry(uint8_t *packetData, uint8_t packetLen, unsigned char ucPrioDelay)
{
	if(_txQueue) {
		if (_state == CD_BACKOFF) {
			if(esp_timer_get_time() < _cdBackoffStart + (LocoNetTickTime * ucPrioDelay)) {
				_state = IDLE;
			} else if(!cdBackoffTimerElapsed()) {
				return LN_CD_BACKOFF;
			} else {
				return LN_PRIO_BACKOFF;
			}
		} else if(_state != IDLE) {
			DEBUG("sendLocoNetPacketTry, state=%d", _state);
			return LN_NETWORK_BUSY;
		}
		LOCONET_TX_LOCK();
		for(uint8_t index = 0; index < packetLen && (_state == IDLE || _state == TX); index++) {
			while(xQueueSendToBack(_txQueue, &packetData[index], (portTickType)5) != pdPASS) {
				esp_task_wdt_reset();
				delay(1);
			}
		}
		LOCONET_TX_UNLOCK();
		// wait for TX to complete
		while(_state == IDLE || _state == TX) {
			esp_task_wdt_reset();
			delay(1);
		}
		if(_state == IDLE || _state == CD_BACKOFF) {
			return LN_DONE;
		} else if(_state == TX_COLLISION) {
			return LN_COLLISION;
		}
	}
	return LN_UNKNOWN_ERROR;
}