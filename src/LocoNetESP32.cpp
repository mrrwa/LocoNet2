#include "LocoNetESP32.h"

#include <functional>
#include <algorithm>
#include "LocoNet.h"
#include "LocoNetSVCV.h"


#include <esp_task_wdt.h>

#include <EEPROM.h>

#define GPIO_DEBUG

#ifdef GPIO_DEBUG
#define DEBUG_IOPIN 14
#define DEBUG_PIN_SB 12
#define DEBUG_PIN_TIMER 13
uint8_t debugPinVal=0;
#endif

#ifdef DEBUG_OUTPUT
char _msg[1024];
char _buf[100];
#undef DEBUG_ISR
#define DEBUG_ISR(...)  do{ snprintf(_buf, 100, __VA_ARGS__); snprintf(_msg, 1024, "%s%s\n", _msg, _buf ); } while(0)
#define DEBUG_ISR_DUMP()  do{ Serial.print(_msg); _msg[0]=0; } while(0);
#endif

static LocoNetESP32 *locoNetInstance = nullptr;

void IRAM_ATTR locoNetTimerCallback() {
    locoNetInstance->loconetBitTimer();
}

void IRAM_ATTR locoNetStartBitCallback() {
    locoNetInstance->loconetStartBit();
}

LocoNetESP32::LocoNetESP32(LocoNetBus *bus, uint8_t rxPin, uint8_t txPin, uint8_t timerId) 
    : LocoNetBackend(bus), _rxPin(rxPin), _txPin(txPin), _timerId(timerId) 
{
    // stash away a pointer to this instance for callback functions
    locoNetInstance = this;
}

bool LocoNetESP32::begin() {
    
    DEBUG("LocoNetESP32.begin");

    _rxQueue = xQueueCreate(32, sizeof(uint8_t));    
    _txQueue = xQueueCreate(32, sizeof(uint8_t));
    
    xTaskCreatePinnedToCore(LocoNetESP32::rxByteProc, "ByteProc", 
        2048, (void* ) this, 2, &_rxByteTask, 1); // cpu1 

    DEBUG("Configuring HW Timer %d as bit timer", _timerId);
    /* Use 1st timer of 4 (counted from zero).
     * Set divider for prescaler (see ESP32 Technical Reference Manual for more
     * info) This should give 10 ticks per bit period for a 16.66kbps link.
     * Assuming clock frequency is 80Mhz, this will be 480.   */
    _lnTimer = timerBegin(_timerId, 480, true);

    /* Attach onTimer function to our timer. */
    timerAttachInterrupt(_lnTimer, locoNetTimerCallback, true);

    /* Set alarm to call onTimer function every bit period. */
    timerAlarmWrite(_lnTimer, 10, true);

    /* set up the TX and RX pins */
    pinMode(_rxPin, INPUT);
    pinMode(_txPin, OUTPUT);
    

#ifdef GPIO_DEBUG
    //pinMode(DEBUG_IOPIN, OUTPUT);
    pinMode(DEBUG_PIN_TIMER, OUTPUT);
    pinMode(DEBUG_PIN_SB, OUTPUT);
    //pinMode(DEBUG_BIT_GEN, OUTPUT);
#endif
    
    timerAlarmEnable(_lnTimer);
    //timerStop(_lnTimer);
    DEBUG("Attaching ISR for RX pin %d", _rxPin);
    changeState(LN_ST_IDLE);

    return true;

}

void LocoNetESP32::end() {
    portENTER_CRITICAL(&_timerMux);
    // shutdown the timer and associated callbacks
    DEBUG("Stopping HW Timer %d", _timerId);
    timerEnd(_lnTimer);

    // remove ISR if present
    enableStartBitISR(false);
    
    portEXIT_CRITICAL(&_timerMux);
}

void LocoNetESP32::enableStartBitISR(bool en) {
    if(en) {
        //DEBUG("Attach the startbit ISR");
        if(_isrAttached) return;
        attachInterrupt(digitalPinToInterrupt(_rxPin), locoNetStartBitCallback,
            (LOCONET_RX_HIGH==HIGH) ? FALLING : RISING);
        _isrAttached = true;
    } else {
        if(!_isrAttached) return;
        //DEBUG("Removing startbit ISR");
        detachInterrupt(digitalPinToInterrupt(_rxPin));
        _isrAttached = false;
    }
}

void IRAM_ATTR LocoNetESP32::changeState(LN_TX_RX_STATUS newStat, Lock lock, uint8_t bit) {
    if(lock==Lock::LOCK) portENTER_CRITICAL(&_timerMux);
    else if(lock==Lock::LOCK_FROM_ISR) portENTER_CRITICAL_ISR(&_timerMux);

    if(newStat == LN_ST_IDLE || newStat == LN_ST_CD_BACKOFF) {
        enableStartBitISR();
    }
    if(newStat == LN_ST_TX || newStat == LN_ST_RX) {
        enableStartBitISR(false);
    }
    
    if(newStat == LN_ST_RX) {
        _lnCurrentRxByte = 0;
    } 

    if(newStat == LN_ST_IDLE) {
        if(timerStarted(_lnTimer)) timerStop(_lnTimer);
    } else {
        //if(!timerStarted(_lnTimer)) { /*timerStart(_lnTimer);*/ 
        timerRestart(_lnTimer);
        //}
    }

    _state = newStat;
    _currentBit = bit;

    if(lock==Lock::LOCK) portEXIT_CRITICAL(&_timerMux);
    else if(lock==Lock::LOCK_FROM_ISR) portEXIT_CRITICAL_ISR(&_timerMux);
}


/**************************************************************************
 *
 * Start Bit Interrupt Routine
 *
 * DESCRIPTION
 * This routine is executed when a falling edge on the incoming serial
 * signal is detected. It disables further interrupts and enables
 * timer to receive the incoming data.
 *
 **************************************************************************/
void IRAM_ATTR LocoNetESP32::loconetStartBit() {
    /* declare a critical section so we can alter the data that is shared
       outside of the iSR */

#ifdef GPIO_DEBUG
    //static uint8_t i=0;
    //digitalWrite(, (i++)%2 );
    digitalWrite(DEBUG_PIN_SB, debugPinVal%2 ); debugPinVal++;
    digitalWrite(DEBUG_PIN_TIMER, LOW );  
#endif

    DEBUG_ISR("StartBit");

    portENTER_CRITICAL_ISR(&_timerMux);

    // make sure we sample the next bit
    timerStop(_lnTimer);
    //timerAlarmWrite(_lnTimer, 15, true); // we try to sample in the middle of bit, 1.5 bits from start bit edge
    timerAlarmWrite(_lnTimer, 6, true); // we try to sample in the middle of bit, 1.5 bits from start bit edge
    changeState(LN_ST_RX, NO_LOCK);

    portEXIT_CRITICAL_ISR(&_timerMux);
}

/**
 * LocoNetESP32::loconetBitTimer()
 * This function gets called every bit time
 * i.e. once every 1/16660 seconds.
 */
void IRAM_ATTR LocoNetESP32::loconetBitTimer() {

    //portENTER_CRITICAL_ISR(&_timerMux);

    // Make sure the timer is set correctly for the next bit
    timerAlarmWrite(_lnTimer, 10, true);

    switch(_state) {
    case LN_ST_IDLE:
        // should not get here
        break;
    case LN_ST_RX: {

        if(_currentBit < 8)  { // 0..7 bits data
            _lnCurrentRxByte >>= 1;
            uint8_t v = digitalRead(_rxPin);
            if(v == LOCONET_RX_HIGH)  {
                _lnCurrentRxByte |= 0x80;
            } 
            DEBUG_ISR("n=%d, v=%d", _currentBit, v);
            _currentBit++;

            #ifdef GPIO_DEBUG
            digitalWrite(DEBUG_PIN_TIMER, v );  
            digitalWrite(DEBUG_PIN_SB, debugPinVal % 2);  debugPinVal++;
            #endif
        } else {
            //DEBUG_ISR("byte is %02x", _lnCurrentRxByte);

            /* The rx pin should not be low at stop bit,
             * if it is then an error has occured on the
             * bus. */
            if(digitalRead(_rxPin) == LOCONET_RX_LOW)  {
                getRxStats()->rxErrors ++;
                _lnCurrentRxByte = 0;
                DEBUG_ISR("Err");
            } else {
                /* Send of the received byte for processing */
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xQueueSendToBackFromISR(_rxQueue, &_lnCurrentRxByte, &xHigherPriorityTaskWoken);
                if(xHigherPriorityTaskWoken==pdTRUE) portYIELD_FROM_ISR();
            }

            #ifdef GPIO_DEBUG
            digitalWrite(DEBUG_PIN_TIMER, HIGH );  
            #endif

            changeState(LN_ST_CD_BACKOFF, Lock::LOCK_FROM_ISR);
            
        }

        break;
    }
        
    case LN_ST_TX:   { // here _currentBit is what bit should be sent this time.

        #ifdef GPIO_DEBUG
        digitalWrite(DEBUG_PIN_SB, debugPinVal % 2);  debugPinVal++;
        #endif

        //DEBUG("bit %d, rx: %d / tx: %d", _currentBit, digitalRead(_rxPin), digitalRead(_txPin)==LOCONET_TX_HIGH);
        if( ( (_currentBit!=0) && checkCollision()) || ( (_currentBit==0) && (digitalRead(_rxPin)==LOCONET_RX_LOW) ))  {
            //DEBUG("rx: %d collides tx: %d", digitalRead(_rxPin), digitalRead(_txPin)==LOCONET_TX_HIGH );
            changeState(LN_ST_TX_COLLISION, Lock::LOCK_FROM_ISR);
            getTxStats()->collisions++;
        } else if(_currentBit == 0) {

            if (uxQueueMessagesWaitingFromISR(_txQueue)) {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xQueueReceiveFromISR(_txQueue, &_lnCurrentTxByte, &xHigherPriorityTaskWoken);
                if(xHigherPriorityTaskWoken==pdTRUE) portYIELD_FROM_ISR();
                
                //DEBUG("txing byte 0x%02x", _lnCurrentTxByte);

                digitalWrite(_txPin, LOCONET_TX_LOW); // start bit

                portENTER_CRITICAL(&_timerMux);
                _currentBit++;
                portEXIT_CRITICAL(&_timerMux);

            }  else  {
                //DEBUG("nothing left");
                changeState(LN_ST_CD_BACKOFF, Lock::LOCK_FROM_ISR);
            }
            
        } else if(_currentBit <= 8)  {
            digitalWrite(_txPin, (_lnCurrentTxByte & 0x01) ? LOCONET_TX_HIGH : LOCONET_TX_LOW );
            //DEBUG("txing bit %d", _lnCurrentTxByte & 0x01);
            /* Get the next bit to transmit */
            _lnCurrentTxByte >>= 1;
            _currentBit++;
        } else if(_currentBit == 9) {
            /* end o'clock  - send the stop bit*/
            digitalWrite(_txPin, LOCONET_TX_HIGH);
            _currentBit = 0; // next step - either send another start bit or start backoff
        } 
        break;
    }

    case LN_ST_CD_BACKOFF: {

        if(_currentBit > LN_BACKOFF_MAX) {
            changeState(LN_ST_IDLE, Lock::LOCK_FROM_ISR); 
            #ifdef DEBUG_ISR_DUMP
            // do slow printf after realtime stuff stops.
            DEBUG_ISR_DUMP();
            #endif
        } else {
            _currentBit++;
        }
        
        break;
    }
    case LN_ST_TX_COLLISION: {
        /* Pull the TX Line low to indicate Collision */
        digitalWrite(_txPin, LOCONET_TX_LOW);

        portENTER_CRITICAL(&_timerMux);

        if(_currentBit == LN_COLLISION_TICKS) {
            digitalWrite(_txPin, LOCONET_TX_HIGH);
            changeState(LN_ST_CD_BACKOFF, Lock::NO_LOCK);
        } else _currentBit++;
        
        /* Re-enable interrupts */
        portEXIT_CRITICAL(&_timerMux);
        break;
    }

    }

}

/**
 * Waits for rx byte from _rxQueue and processes it. Loops infinitely.
 */
void LocoNetESP32::rxByte() {

    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200);

    uint8_t rx;

    while(1) {

        if(xQueueReceive(_rxQueue, &rx, xMaxBlockTime) == pdTRUE) {
            DEBUG("RXed %02x", rx);
            consume(rx);
        }

        yield();
    }

}

/**
 * LocoNetESP32Class::checkCollision()
 *
 * Checks if there is a collision on the Loconet bus,
 * This happens when the value we put into the tx
 * register does not match that in the RX Register.
 *
 * @return true if there was a collision, false otherwise
 */
bool LocoNetESP32::checkCollision() {
    
    if ( (int)LOCONET_RX_HIGH != (int)LOCONET_TX_HIGH) {
        return ((digitalRead(_rxPin) & 0x1) == (digitalRead(_txPin) & 0x1));
    } else {
        return ((digitalRead(_rxPin) & 0x1) != (digitalRead(_txPin) & 0x1));
    }
}


/**
 * LocoNetESP32::sendLocoNetPacketTry
 *
 * Attempts to send a loconet packet.
 *
 * @param txData - a pointer to an lnMsg packet to transmit
 * @param ucPrioDelay - the delay to add to wait for the bus
 *                      to remain clear before transmission
 * @return LN_STATUS - the current status of the Loconet
 *                   transmission.
 */
LN_STATUS LocoNetESP32::sendLocoNetPacketTry(uint8_t *packetData, uint8_t packetLen, unsigned char ucPrioDelay) {
    
    xQueueReset(_txQueue);
    for(uint8_t i=0; i<packetLen; i++)
        if( xQueueSendToBack(_txQueue, &packetData[i], (TickType_t) 0 )  != pdPASS ) {
            return LN_UNKNOWN_ERROR;
        }
    DEBUG("Queued %d bytes for TX", packetLen);

    // check if we are in Priority backoff that we can interrupt
    if(_state == LN_ST_CD_BACKOFF) {
        if(_currentBit >= ucPrioDelay) {
            DEBUG("Switching to IDLE state");
            changeState(LN_ST_IDLE);
        }
    }
    // check if we are in CD backoff or Priority backoff
    if(_state == LN_ST_CD_BACKOFF) {
        if(_currentBit < LN_CARRIER_TICKS) {
            return LN_CD_BACKOFF;
        } else {
            return LN_PRIO_BACKOFF;
        }
    }

    if(_state != LN_ST_IDLE) {
        // neither idle nor backoff -> busy
        return LN_NETWORK_BUSY;
    }

    timerAlarmWrite(_lnTimer, 10, true);

    changeState(LN_ST_TX);
    
    while(_state == LN_ST_TX) {
        // now busy wait until the interrupts do the rest of the transmitting
        esp_task_wdt_reset();
        delayMicroseconds(100);
    }
    if(_state == LN_ST_CD_BACKOFF || _state == LN_ST_IDLE) {
        txStats.txPackets++;
        return LN_DONE;
    } else if(_state == LN_ST_TX_COLLISION) {
        return LN_COLLISION;
    }
    return LN_UNKNOWN_ERROR; // everything else is an error
}

uint8_t LocoNetSystemVariable::readSVStorage(uint16_t offset) {
    if(offset == SV_ADDR_EEPROM_SIZE) {
        return 0xFF;
    } else if(offset == SV_ADDR_SW_VERSION) {
        return _swVersion;
    }
    offset -= 2;    // Map SV Address to EEPROM Offset - Skip SV_ADDR_EEPROM_SIZE & SV_ADDR_SW_VERSION
    return EEPROM.read(offset);
}

uint8_t LocoNetSystemVariable::writeSVStorage(uint16_t offset, uint8_t value) {
    offset -= 2;      // Map SV Address to EEPROM Offset - Skip SV_ADDR_EEPROM_SIZE & SV_ADDR_SW_VERSION
    uint8_t oldValue = EEPROM.read(offset);
    if(oldValue != value) {
        EEPROM.write(offset, value);
        if(_svChangeCallback) {
            _svChangeCallback(offset + 2, value, oldValue);
        }
    }
    return EEPROM.read(offset);
}

void LocoNetSystemVariable::reconfigure() {
    // shutdown the LocoNet subsystems (required for ESP32 restart is a soft restart and ISRs will not be shutdown)
    _locoNet.end();
    if(_reconfigureCallback) {
        _reconfigureCallback();
    }
    // reset the esp32
    esp_restart();
}