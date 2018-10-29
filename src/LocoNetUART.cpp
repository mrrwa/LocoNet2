#include "LocoNetUART.h"
#ifndef ESP32
void LocoNetUartClass::init()
{
}

LN_STATUS LocoNetUartClass::sendLocoNetPacketTry(lnMsg *txData, unsigned char ucPrioDelay)
{
	txData = txData;						// Keep the Compilar happy
	ucPrioDelay = ucPrioDelay;
	
	return LN_DONE;
}

#endif