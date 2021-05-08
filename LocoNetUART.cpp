#include "LocoNetUART.h"

void LocoNetUartClass::init()
{
}

LN_STATUS LocoNetUartClass::sendLocoNetPacketTry(lnMsg *txData, unsigned char ucPrioDelay)
{
	txData = txData;						// Keep the Compilar happy
	ucPrioDelay = ucPrioDelay;
	
	return LN_DONE;
}

void LocoNetUartClass::debug(int32_t Data)
{
	
}	

void LocoNetUartClass::debug(const char* Data) 
{
	
}

void LocoNetUartClass::debug(uint32_t Data, uint8_t Base)
{
	
}