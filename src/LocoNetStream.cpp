#include "LocoNetStream.h"

LN_STATUS LocoNetStream::sendLocoNetPacketTry(uint8_t *packetData, uint8_t packetLen, unsigned char ucPrioDelay)
{
	DEBUG("sendLocoNetPacketTry: begin state: %s", getStatusStr(_state));
	
	if (_state == LN_CD_BACKOFF)
	{
			// check if we've already waited for the maximum CD_BACKOFF delay + the PRIORITY delay
		if(hasCDBackoffTimerExpired(ucPrioDelay))
			_state = LN_IDLE;

			// if not then have we at least waited for the minimum CD_BACKOFF delay 			
		else if(!hasCDBackoffTimerExpired())
			return LN_CD_BACKOFF;

			// Well we must now be in-between the minimum CD_BACKOFF delay and the PRIORITY delay    			
		else	
			return LN_PRIO_BACKOFF;
	}
	
	else if(_state == LN_COLLISION && hasCollisionTimerExpired())
	{
		_state = LN_CD_BACKOFF;
	}

	if(_state != LN_IDLE)
	{
		DEBUG("sendLocoNetPacketTry: return %s", getStatusStr(_state));
		return _state;
	}
	
		// Do a process() just in case another byte has arrived after we last checked 
	process();
	
	if( !isBusy() )
	{
		uint32_t tempRxThreshold;
		beforeSend();
		
		DEBUG("sendLocoNetPacketTry: Start to send data");
		while(packetLen--)
		{
			_serialPort->write(packetData, 1);
			
			uint64_t startMillis = millis();
			int inByte = _serialPort->read();
			while(inByte == -1 and ((millis() - startMillis) < 2) )
				inByte = _serialPort->read();
			
			if(inByte != *packetData)
			{
				DEBUG("sendLocoNetPacketTry: Collision Detected  Tx: %0x  Rx: %0x", *packetData, inByte);
				
				startCollisionTimer();
					
				txStats.collisions++;
				sendBreak();

				afterSend();
					
				return LN_COLLISION;
			}

			packetData++;
		}
		txStats.txPackets++;
		
		startCDBackoffTimer();
		
		afterSend();

		return LN_IDLE;
	}

	return LN_NETWORK_BUSY;
}

void LocoNetStream::begin(Stream * serialPort)
{
	_serialPort = serialPort;
}

void LocoNetStream::end()
{}

void LocoNetStream::process()
{
	if(_serialPort->available())
	{
		DEBUG("process: Process LocoNet Bytes");
		while(_serialPort->available())
			consume((uint8_t)_serialPort->read());
			
		startCDBackoffTimer();
	}
	
	else if(_state == LN_CD_BACKOFF && hasCDBackoffTimerExpired())
	{
		DEBUG("process: Switching to IDLE");
		_state = LN_IDLE;
	}
	
	else if(_state == LN_COLLISION && hasCollisionTimerExpired())
	{
		DEBUG("process: Switching to CD_BACKOFF");
		_state = LN_CD_BACKOFF;
	}
	
	if(_state == LN_IDLE && isBusy())
	{
		DEBUG("process: LocoNet Active");
		startCDBackoffTimer();
	}
}

void LocoNetStream::startCollisionTimer() {
	DEBUG("startCollisionTimer");
	_serialPort->flush();
	_state = LN_COLLISION;
	txStats.collisions++;
	_collisionTimeout = micros() + CollisionTimeoutIncrement;
}

bool LocoNetStream::hasCollisionTimerExpired()
{
	return _collisionTimeout <= micros();
}

void LocoNetStream::startCDBackoffTimer()
{
	DEBUG("startCDBackoffTimer");
	_state = LN_CD_BACKOFF;
	_cdBackoffStart = micros();
	_cdBackoffTimeout = _cdBackoffStart + CDBackoffTimeoutIncrement;
}

bool LocoNetStream::hasCDBackoffTimerExpired(uint8_t PrioDelay)
{
	return (_cdBackoffTimeout + (PrioDelay * LocoNetTickTime)) <= micros();
}