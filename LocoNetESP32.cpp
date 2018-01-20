#include <stdio.h>
#include "LocoNet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "LocoNetESP32.h"

#define GPIO_DEBUG  /* enable this if you have a logic analyser and wish to debug the iSR*/

#define SW_RX_PIN 16
#define SW_TX_PIN 15

#define LOCONET_TX_LOW 0  /* Invert these if required*/
#define LOCONET_TX_HIGH 1

#define LOCONET_RX_LOW 0  /* Invert these if required*/
#define LOCONET_RX_HIGH 1

#ifdef GPIO_DEBUG
#define DEBUG_IOPIN 12
#define DEBUG_BIT_TIMER 13
#define DEBUG_SB 14
#define DEBUG_BIT_GEN 26
#endif

#define BUF_SIZE (256)

hw_timer_t * timer = NULL;

volatile SemaphoreHandle_t timerSemaphore;
static TaskHandle_t xTaskToNotify = NULL;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint8_t lnTxSuccess;   // this boolean flag as a message from timer interrupt to send function
volatile uint8_t lnState;
volatile uint8_t lnCurrentRxByte;
volatile uint8_t lnBitCount;

void LocoNetESP32Class::init()
{

    Serial.println("LocoNetESP32Class Initialising");

    /* Create semaphore to inform us when the timer has fired */
    timerSemaphore = xSemaphoreCreateBinary();

    xTaskCreate(LocoNetESP32Class::locoNetESP32Task, "LocoNetESP32Task", 1600, (void* ) this, 2, &xTaskToNotify);

    /* Use 1st timer of 4 (counted from zero).
     * Set divider for prescaler (see ESP32 Technical Reference Manual for more
     * info) This  should give 10 ticks per bit period for a 16.66kbps link.
     * Assuming clock frequency is 80Mhz, this will be 480    */
    timer = timerBegin(0, 480, true);

    /* Attach onTimer function to our timer. */
    timerAttachInterrupt(timer, &loconetBitTimer, true);

    /* Set alarm to call onTimer function every bit period. */
    timerAlarmWrite(timer, 10, true);

    /* Start the timer. */
    timerAlarmEnable(timer);

    /* get the pointer to the rx stats */
    rxStats = getRxStats();

    /* set up the TX and RX pins */
    pinMode(SW_RX_PIN, INPUT_PULLUP);
    pinMode(SW_TX_PIN, OUTPUT);

#ifdef GPIO_DEBUG
    pinMode(DEBUG_IOPIN, OUTPUT);
    pinMode(DEBUG_BIT_TIMER, OUTPUT);
    pinMode(DEBUG_SB, OUTPUT);
    pinMode(DEBUG_BIT_GEN, OUTPUT);
#endif

    /* attach the startbit interrup to the rx pin */
    attachInterrupt(digitalPinToInterrupt(SW_RX_PIN), LocoNetESP32Class::loconetStartBit, RISING); //FALLING for real hardware
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
void IRAM_ATTR LocoNetESP32Class::loconetStartBit()
{
    /*declare a critical section so we can alter the data that is shared
     outside of the iSR*/
    portENTER_CRITICAL_ISR(&timerMux);

#ifdef GPIO_DEBUG
    digitalWrite(DEBUG_IOPIN, 0);
#endif

    /* Disable the Start Bit interrupt */
    detachInterrupt(digitalPinToInterrupt(SW_RX_PIN));

    /* make sure we sample the next bit */
    timerAlarmWrite(timer, 6, true);
    timerRestart(timer);

    /* Set the State to indicate that we have begun to Receive */
    lnState = LN_ST_RX;

    /* Reset the bit counter so that on first increment it is on 0 */
    lnBitCount = 0;
    lnCurrentRxByte = 0;
    portEXIT_CRITICAL_ISR(&timerMux);
}

/**
 * LocoNetESP32Class::loconetBitTimer()
 * This function gets called every bit time
 * i.e. once every 1/16660 seconds.*/
void IRAM_ATTR LocoNetESP32Class::loconetBitTimer()
{
#ifdef GPIO_DEBUG
    static uint8_t i = 0;
#endif
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Increment the counter and set the time of ISR */
    portENTER_CRITICAL_ISR(&timerMux);

    /* Make sure the timer is set correctly for the next round */
    timerAlarmWrite(timer, 10, true);

    lnBitCount++;

#ifdef GPIO_DEBUG
    digitalWrite(DEBUG_BIT_TIMER, i % 2);
    i++;
#endif

    /* Notify the task that the one bit time has occured. */
    vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);

    /* Re-enable interrupts */
    portEXIT_CRITICAL_ISR(&timerMux);
    portYIELD_FROM_ISR( );

}

/**
 * LocoNetESP32Class::TaskRun()
 * This task handles the main part of the TX and RX on the
 * Loconet bus. It sits in its own thread, and mostly does
 * nothing until it is able to get the timerSemaphore that
 * indicates another bit period has occurred,
 */
void LocoNetESP32Class::TaskRun()
{
#ifdef GPIO_DEBUG
    static uint8_t i = 0;
#endif
    uint32_t ulNotificationValue;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200);

    /* Wait for the bit timer to kick the task */
    ulNotificationValue = ulTaskNotifyTake( pdTRUE, xMaxBlockTime);

    if(ulNotificationValue == 1)
    {

#ifdef GPIO_DEBUG
        digitalWrite(DEBUG_BIT_TIMER, i % 2);
        i++;
#endif

        if(lnState == LN_ST_RX)
        {

#ifdef GPIO_DEBUG
            if(lnBitCount == 1)
            {
                digitalWrite(DEBUG_SB, 1);
            }
#endif

            if(lnBitCount < 10)
            {
                lnCurrentRxByte >>= 1;
                if(digitalRead(SW_RX_PIN) == LOCONET_RX_HIGH)
                {
                    lnCurrentRxByte |= 0x80;
#ifdef GPIO_DEBUG
                    digitalWrite(DEBUG_IOPIN, 0);
                }
                else
                {
                    digitalWrite(DEBUG_IOPIN, 1);
#endif
                }
            }
            else
            {
#ifdef GPIO_DEBUG
                digitalWrite(DEBUG_SB, 0);
                digitalWrite(DEBUG_IOPIN, 0);

#endif

                /* The rx pin should not be low at this point,
                 * if it is then an error has occured on the
                 * bus.*/
                if(digitalRead(SW_RX_PIN) == LOCONET_RX_LOW)
                {
                    errCount++;
                    lnCurrentRxByte = 0;
                }
                else
                {
                    /* Send of the received byte for processing */
                    process(lnCurrentRxByte);
                }

                /* disable interrupts whilst the lnBitCount is reset */
                portENTER_CRITICAL(&timerMux);
                lnBitCount = 0;
                lnState = LN_ST_CD_BACKOFF;
                /* Re-enable interrupts */
                portEXIT_CRITICAL(&timerMux);

                /* re-attach the start bit interrupt */
                attachInterrupt(digitalPinToInterrupt(SW_RX_PIN), LocoNetESP32Class::loconetStartBit, FALLING);
            }
            yield();

        }

        if(lnState == LN_ST_TX)
        {
            /* Are we in the TX State?
             * To get to this point we have already begun the TX cycle so we need to
             first check for a Collision. */
            if(CheckCollision())
            {
                /* Disable interrupts whilst the lnState is changed
                 * and the lnBitCount is reset. */
                portENTER_CRITICAL(&timerMux);
                lnBitCount = 0;
                lnState = LN_ST_TX_COLLISION;
                /* Re-enable interrupts */
                portEXIT_CRITICAL(&timerMux);
            }
            else if(lnBitCount <= 1)
            {
                /* Send the start bit */
                digitalWrite(SW_TX_PIN, LOCONET_TX_LOW);
            }
            else if(lnBitCount <= 9)
            {
                if(*lnCurrentTxBytePtr & 0x01)
                {
                    digitalWrite(SW_TX_PIN, LOCONET_TX_HIGH);
#ifdef GPIO_DEBUG
                    digitalWrite(DEBUG_IOPIN, 0);
#endif
                }
                else
                {
                    digitalWrite(SW_TX_PIN, LOCONET_TX_LOW);
#ifdef GPIO_DEBUG
                    digitalWrite(DEBUG_IOPIN, 1);
#endif
                }
                /* Get the next bit to transmit */
                *lnCurrentTxBytePtr >>= 1;
            }
            else if(lnBitCount == 10)
            {
                /* end o'clock  - send the stop bit*/
                digitalWrite(SW_TX_PIN, LOCONET_TX_HIGH);
            }
            else if(txBytesRemaining > 0)
            {
                /* If there are still bytes to transmit
                 * get the next byte   */
                txBytesRemaining--;
                lnCurrentTxBytePtr++;

                portENTER_CRITICAL(&timerMux);
                /* Setup lnBitCount for the next byte*/
                lnBitCount = 0;
                /* Re-enable interrupts */
                portEXIT_CRITICAL(&timerMux);

            }
            else
            {
                /* disable interrupts whilst we write to lnState ,
                 * and clear the lnBitCount. */
                portENTER_CRITICAL(&timerMux);
                /* Begin CD Backoff state */
                lnBitCount = 0;
                lnState = LN_ST_CD_BACKOFF;
                /* Re-enable interrupts */
                portEXIT_CRITICAL(&timerMux);
            }

        }

        if(lnState == LN_ST_TX_COLLISION)
        {
            /* Pull the TX Line low to indicate Collision */
            digitalWrite(SW_TX_PIN, LOCONET_TX_LOW);

            /* disable interrupts whilst we write to lnState ,
             * modify the timer period, and clear the lnBitCount. */
            portENTER_CRITICAL(&timerMux);

            /* Wait for 15 bit periods due to the collision */
            timerAlarmWrite(timer, 150, true);
            lnBitCount = 0;

            /* Next time through we will be in backoff. */
            lnState = LN_ST_CD_BACKOFF;
            /* Re-enable interrupts */
            portEXIT_CRITICAL(&timerMux);
            yield();
        }

        if(lnState == LN_ST_CD_BACKOFF)
        {
            if(lnBitCount == 0)
            {
                /* Re-enable the start bit detection now that backoff is active */
                attachInterrupt(digitalPinToInterrupt(SW_RX_PIN), LocoNetESP32Class::loconetStartBit, FALLING);
            }
            else if(lnBitCount >= LN_BACKOFF_MAX)
            {
                /* disable interrupts whilst we write to lnState */
                portENTER_CRITICAL(&timerMux);
                /* declare network to free after maximum backoff delay */
                lnState = LN_ST_IDLE;
                /* Re-enable interrupts after lnState has been modified */
                portEXIT_CRITICAL(&timerMux);
            }
            else
            {
                /*do nothing */
                yield();
            }
        }
        if(rxStats != 0)
        {
            rxStats->rxErrors += errCount;
            errCount = 0;
        }
    }
    yield();
}

/**
 * LocoNetESP32Class::CheckCollision()
 *
 * Checks if there is a collision on the Loconet bus,
 * This happens when the value we put into the tx
 * register does not match that in the RX Register.
 *
 * @return true if there was a collision, false otherwise
 */
bool LocoNetESP32Class::CheckCollision()
{
#if LOCONET_RX_HIGH != LOCONET_TX_HIGH
    return ((digitalRead(SW_RX_PIN) & 0x1) == (digitalRead(SW_TX_PIN) & 0x1));
#else
    return ((digitalRead(SW_RX_PIN) & 0x1) != (digitalRead(SW_TX_PIN) & 0x1));
#endif

}

/**
 * LocoNetESP32Class::sendLocoNetPacketTry
 *
 * Attempts to send a loconet packet.
 *
 * @param txData - a pointer to an lnMsg packet to transmit
 * @param ucPrioDelay - the delay to add to wait for the bus
 *                      to remain clear before transmission
 * @return LN_STATUS - the current status of the Loconet
 *                   transmission.
 */
LN_STATUS LocoNetESP32Class::sendLocoNetPacketTry(lnMsg *txData, unsigned char ucPrioDelay)
{
    uint8_t CheckSum;
    uint8_t CheckLength;

    txBytesRemaining =
            ((txData->sz.command & (uint8_t)0x60) == (uint8_t)0x60) ?
                    txData->sz.mesg_size : ((txData->sz.command & (uint8_t)0x60) >> (uint8_t)4) + 2;
    if(txBytesRemaining > LN_BUF_SIZE)
    {
        txBytesRemaining = LN_BUF_SIZE;
    }

    memcpy(txBuffer, txData, txBytesRemaining);

    /* First calculate the checksum as it may not have been done */
    CheckLength = txBytesRemaining - 1;
    CheckSum = 0xFF;

    for(lnTxIndex = 0; lnTxIndex < CheckLength; lnTxIndex++)
    {
        CheckSum ^= txBuffer[lnTxIndex];
    }

    txBuffer[CheckLength] = CheckSum;

    /* clip maximum prio delay */
    if(ucPrioDelay > LN_BACKOFF_MAX)
    {
        ucPrioDelay = LN_BACKOFF_MAX;
    }

    /* Load the first Byte */
    lnCurrentTxBytePtr = &txBuffer[0];

    if(lnState == LN_ST_CD_BACKOFF)
    {
        if(lnBitCount >= ucPrioDelay)
        {
            portENTER_CRITICAL(&timerMux);
            lnState = LN_ST_IDLE;
            portEXIT_CRITICAL(&timerMux);
        }
        else if(lnBitCount < LN_CARRIER_TICKS)
        {
            return LN_CD_BACKOFF;
        }
        else
        {
            return LN_PRIO_BACKOFF;
        }
    }

    if(lnState != LN_ST_IDLE)
    {
        /* neither idle nor backoff -> busy */
        return LN_NETWORK_BUSY;
    }

    /* disable interrupts whilst we clear the lnBitCount and
     * Set the State to Transmit . */
    portENTER_CRITICAL(&timerMux);
    lnBitCount = 0;
    lnState = LN_ST_TX;
    portEXIT_CRITICAL(&timerMux);

    /* Disable the start bit interrupt, we don't want to think
     * our TX is an RX.     */
    detachInterrupt(digitalPinToInterrupt(SW_RX_PIN));

    while(lnState == LN_ST_TX)
    {
        /* now busy - wait until the interrupts do the rest of the transmitting */
    }
    if((lnState == LN_ST_CD_BACKOFF) || (lnState == LN_ST_IDLE))
    {
        return LN_DONE;
    }
    if(lnState == LN_ST_TX_COLLISION)
    {
        return LN_COLLISION;
    }
    return LN_UNKNOWN_ERROR; // everything else is an error
}
