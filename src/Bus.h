/**
 * Takes LocoNet messages from several sources and resends them to
 * many destinations. Used for resending messages between LocoNet,
 * Loconet over USB-Serial and LbServer.
 * */

#pragma once

#if defined(ARDUINO)
  // The <Embedded_Template_Library.h> try to redefine ARDUINO_BOARD
  // so the preprocessor code below should suppress the warning 
  //
  // Save the compiler-injected board name to restore later
  #ifdef ARDUINO_BOARD
    #define ORIGINAL_ARDUINO_BOARD ARDUINO_BOARD
    #undef ARDUINO_BOARD
  #endif

  #include <Embedded_Template_Library.h> // Mandatory for Arduino IDE only
  
  // Restore the original board definition
  #ifdef ORIGINAL_ARDUINO_BOARD
    #undef ARDUINO_BOARD
    #define ARDUINO_BOARD ORIGINAL_ARDUINO_BOARD
  #endif  
#endif
#include <etl/vector.h>

#include "ln_opc.h"

#define BUS_DEBUG_

#ifdef BUS_DEBUG
    #include <Arduino.h>
    #define BUS_DEBUGF(format, ...)  do{ log_printf(ARDUHAL_LOG_FORMAT(I, format), ##__VA_ARGS__); }while(0)
#else
    #define BUS_DEBUGF(...)
#endif

template < class Msg, class Ret >
class Consumer
{
public:
    virtual Ret onMessage (const Msg& msg) = 0;
};

template <class Msg, class Ret, Ret okVal, const size_t MAX_CONSUMERS>
class Bus
{
public:
    using MsgConsumer = Consumer<Msg, Ret>;

    Ret broadcast (const Msg &msg, MsgConsumer* sender = nullptr)
    {

        BUS_DEBUGF ("message %02x %02x...", msg.data[0], msg.data[1]);

        Ret ret = okVal;
        for (const auto & c: consumers)
        {
            if (c!=sender)
            {
                Ret v = c->onMessage (msg);
                if (v!=okVal) ret = v;
            }
        }
        return ret;
    }

    void addConsumer (MsgConsumer * c)
    {
        consumers.push_back (c);
    }

    void removeConsumer (MsgConsumer * c)
    {
        consumers.erase (etl::remove (consumers.begin(), consumers.end(), c), consumers.end());
    }

private:
    etl::vector<MsgConsumer*, MAX_CONSUMERS> consumers;
};

