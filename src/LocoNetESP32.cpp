#include <Arduino.h>
#include <functional>
#include <algorithm>
#include "LocoNet.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "LocoNetESP32.h"

#include <EEPROM.h>

#define BUF_SIZE (256)

static LocoNetESP32 *locoNetInstance = nullptr;
void locoNetTaskCallback(void *pvParams) {
    locoNetInstance->TaskRun();
}

void locoNetTimerCallback() {
    locoNetInstance->loconetBitTimer();
}

void locoNetStartBitCallback() {
    locoNetInstance->loconetStartBit();
}

LocoNetESP32::LocoNetESP32(uint8_t rxPin, uint8_t txPin, uint8_t timerId) : LocoNet(), _rxPin(rxPin), _txPin(txPin), _timerId(timerId) {
    // stash away a pointer to this instance for callback functions
    locoNetInstance = this;
}

void LocoNetESP32::init() {
    // locoNetTaskCallback
    xTaskCreate(locoNetTaskCallback, "LocoNetESP32Task", 1600, nullptr, 2, &xTaskToNotify);

    /* Use one of the four ESP32 hardware timers.
     * Set divider for prescaler (see ESP32 Technical Reference Manual for more
     * info) This  should give 10 ticks per bit period for a 16.66kbps link.
     * Assuming clock frequency is 80Mhz, this will be 480    */
    _lnTimer = timerBegin(_timerId, 480, true);

    /* Attach onTimer function to our timer. */
    timerAttachInterrupt(_lnTimer, locoNetTimerCallback, true);

    /* Set alarm to call onTimer function every bit period. */
    timerAlarmWrite(_lnTimer, 10, true);

    /* Start the timer. */
    timerAlarmEnable(_lnTimer);
    
    /* set up the TX and RX pins */
    pinMode(_rxPin, INPUT_PULLUP);
    pinMode(_txPin, OUTPUT);

    /* attach the startbit interrup to the rx pin */
    attachInterrupt(digitalPinToInterrupt(_rxPin), locoNetStartBitCallback, RISING); //FALLING for real hardware
}

/**************************************************************************
 *
 * Start Bit Interrupt Routine
 *
 * DESCRIPTION
 * This routine is executed when a falling edge on the incoming serial
 * signal is detected. It disables further interrupts and enables
 * timer interrupts (bit-timer) because the UART must now receive the
 * incoming data.
 *
 **************************************************************************/
void IRAM_ATTR LocoNetESP32::loconetStartBit() {
    /*declare a critical section so we can alter the data that is shared
     outside of the iSR*/
    portENTER_CRITICAL_ISR(&timerMux);

    /* Disable the Start Bit interrupt */
    detachInterrupt(digitalPinToInterrupt(_rxPin));

    /* make sure we sample the next bit */
    timerAlarmWrite(_lnTimer, 6, true);
    timerRestart(_lnTimer);

    /* Set the State to indicate that we have begun to Receive */
    lnState = LN_ST_RX;

    /* Reset the bit counter so that on first increment it is on 0 */
    lnBitCount = 0;
    lnCurrentRxByte = 0;
    portEXIT_CRITICAL_ISR(&timerMux);
}

/**
 * LocoNetESP32::loconetBitTimer()
 * This function gets called every bit time
 * i.e. once every 1/16660 seconds.*/
void IRAM_ATTR LocoNetESP32::loconetBitTimer() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Increment the counter and set the time of ISR */
    portENTER_CRITICAL_ISR(&timerMux);

    /* Make sure the timer is set correctly for the next round */
    timerAlarmWrite(_lnTimer, 10, true);

    lnBitCount++;

    /* Notify the task that the one bit time has occured. */
    vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);

    /* Re-enable interrupts */
    portEXIT_CRITICAL_ISR(&timerMux);
}

#define BLOCK_ISR_FOR_BIT_STATE_CHANGE(newBitCount, newState) \
portENTER_CRITICAL(&timerMux); \
lnBitCount = newBitCount; \
lnState = newState; \
portEXIT_CRITICAL(&timerMux);

#define BLOCK_ISR_FOR_BIT_CHANGE(newBitCount) \
portENTER_CRITICAL(&timerMux); \
lnBitCount = newBitCount; \
portEXIT_CRITICAL(&timerMux);

#define BLOCK_ISR_FOR_STATE_CHANGE(newState) \
portENTER_CRITICAL(&timerMux); \
lnState = newState; \
portEXIT_CRITICAL(&timerMux);

/**
 * LocoNetESP32::TaskRun()
 * This task handles the main part of the TX and RX on the
 * Loconet bus. It sits in its own thread, and mostly does
 * nothing until it is able to get the timerSemaphore that
 * indicates another bit period has occurred,
 */
void LocoNetESP32::TaskRun() {
    while(true) {
        if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200))) {
            if(lnState == LN_ST_RX) {
                if(lnBitCount < 10) {
                    lnCurrentRxByte >>= 1;
                    if(digitalRead(_rxPin) == LOCONET_RX_HIGH) {
                        lnCurrentRxByte |= 0x80;
                    }
                } else {
                    /* The rx pin should not be low at this point,
                     * if it is then an error has occured on the
                     * bus.*/
                    if(digitalRead(_rxPin) == LOCONET_RX_LOW) {
                        rxBuffer.stats.rxErrors++;
                        lnCurrentRxByte = 0;
                    } else {
                        /* Send of the received byte for processing */
                        process(lnCurrentRxByte);
                    }
                    BLOCK_ISR_FOR_BIT_STATE_CHANGE(0, LN_ST_CD_BACKOFF)
                    /* re-attach the start bit interrupt */
                    attachInterrupt(digitalPinToInterrupt(_rxPin), locoNetStartBitCallback, FALLING);
                }
            }
            if(lnState == LN_ST_TX) {
                /* Are we in the TX State?
                 * To get to this point we have already begun the TX cycle so we need to
                 * first check for a Collision. */
                if(CheckCollision()) {
                    /* Disable interrupts whilst the lnState is changed
                     * and the lnBitCount is reset. */
                    BLOCK_ISR_FOR_BIT_STATE_CHANGE(0, LN_ST_TX_COLLISION)
                } else if(lnBitCount <= 1) {
                    /* Send the start bit */
                    digitalWrite(_txPin, LOCONET_TX_LOW);
                } else if(lnBitCount <= 9) {
                    if(lnCurrentTxByte & 0x01) {
                        digitalWrite(_txPin, LOCONET_TX_HIGH);
                    } else {
                        digitalWrite(_txPin, LOCONET_TX_LOW);
                    }
                    /* Get the next bit to transmit */
                    lnCurrentTxByte >>= 1;
                } else if(lnBitCount == 10) {
                    /* end o'clock  - send the stop bit*/
                    digitalWrite(_txPin, LOCONET_TX_HIGH);
                } else if(!txBuffer.empty()) {
                    /* If there are still bytes to transmit
                     * get the next byte   */
                    txBuffer.erase(txBuffer.cbegin());
                    lnCurrentTxByte = txBuffer[0];
                    BLOCK_ISR_FOR_BIT_CHANGE(0)
                } else {
                    BLOCK_ISR_FOR_BIT_STATE_CHANGE(0, LN_ST_CD_BACKOFF)
                }
            }

            if(lnState == LN_ST_TX_COLLISION) {
                /* Pull the TX Line low to indicate Collision */
                digitalWrite(_txPin, LOCONET_TX_LOW);

                BLOCK_ISR_FOR_BIT_STATE_CHANGE(0, LN_ST_CD_BACKOFF)
                /* Wait for 15 bit periods due to the collision */
                timerAlarmWrite(_lnTimer, 150, true);
            }

            if(lnState == LN_ST_CD_BACKOFF) {
                if(lnBitCount == 0) {
                    /* Re-enable the start bit detection now that backoff is active */
                    attachInterrupt(digitalPinToInterrupt(_rxPin), locoNetStartBitCallback, FALLING);
                } else if(lnBitCount >= LN_BACKOFF_MAX) {
                    BLOCK_ISR_FOR_STATE_CHANGE(LN_ST_IDLE)
                } 
            }
        }
        taskYIELD();
    }
}

/**
 * LocoNetESP32::CheckCollision()
 *
 * Checks if there is a collision on the Loconet bus,
 * This happens when the value we put into the tx
 * register does not match that in the RX Register.
 *
 * @return true if there was a collision, false otherwise
 */
bool LocoNetESP32::CheckCollision() {
#if LOCONET_RX_HIGH != LOCONET_TX_HIGH
    return ((digitalRead(_rxPin) & 0x1) == (digitalRead(_txPin) & 0x1));
#else
    return ((digitalRead(_rxPin) & 0x1) != (digitalRead(_txPin) & 0x1));
#endif
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
    std::copy(&packetData[0], &packetData[packetLen], back_inserter(txBuffer));

    /* clip maximum prio delay */
    if(ucPrioDelay > LN_BACKOFF_MAX) {
        ucPrioDelay = LN_BACKOFF_MAX;
    }

    /* Load the first Byte */
    lnCurrentTxByte = txBuffer[0];

    if(lnState == LN_ST_CD_BACKOFF) {
        if(lnBitCount >= ucPrioDelay) {
            BLOCK_ISR_FOR_STATE_CHANGE(LN_ST_IDLE)
        } else if(lnBitCount < LN_CARRIER_TICKS) {
            return LN_CD_BACKOFF;
        } else {
            return LN_PRIO_BACKOFF;
        }
    }

    if(lnState != LN_ST_IDLE) {
        /* neither idle nor backoff -> busy */
        return LN_NETWORK_BUSY;
    }

    BLOCK_ISR_FOR_BIT_STATE_CHANGE(0, LN_ST_TX)

    /* Disable the start bit interrupt, we don't want to think
     * our TX is an RX.     */
    detachInterrupt(digitalPinToInterrupt(_rxPin));

    while(lnState == LN_ST_TX) {
        /* now busy - wait until the interrupts do the rest of the transmitting */
    }
    if(lnState == LN_ST_CD_BACKOFF || lnState == LN_ST_IDLE) {
        return LN_DONE;
    } else if(lnState == LN_ST_TX_COLLISION) {
        return LN_COLLISION;
    }
    return LN_UNKNOWN_ERROR; // everything else is an error
}

uint8_t LocoNetSystemVariable::readSVStorage(uint16_t Offset ) {
    if( Offset == SV_ADDR_EEPROM_SIZE) {
        return 0xFF;
    } else if( Offset == SV_ADDR_SW_VERSION ) {
        return swVersion ;
    } else {
        Offset -= 2;    // Map SV Address to EEPROM Offset - Skip SV_ADDR_EEPROM_SIZE & SV_ADDR_SW_VERSION
        return EEPROM.read((int)Offset);
  }
}

uint8_t LocoNetSystemVariable::writeSVStorage(uint16_t Offset, uint8_t Value) {
    Offset -= 2;      // Map SV Address to EEPROM Offset - Skip SV_ADDR_EEPROM_SIZE & SV_ADDR_SW_VERSION
    if( EEPROM.read((int)Offset) != Value )
    {
        EEPROM.write((int)Offset, Value);
        if(notifySVChanged)
            notifySVChanged(Offset+2);
    }
    return EEPROM.read((int)Offset);
}