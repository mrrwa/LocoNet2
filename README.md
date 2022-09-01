# LocoNet2
A refactor of the original [LocoNet](https://github.com/mrrwa/LocoNet) library to be much more object oriented, with derived classes for each different hardware interface

Currently this only supports the ESP32 processor.

## positron96/LocoNet2

This is a modified version of LocoNet2 library that is tailored for command station use. It is now used in the [ESP32 LocoNet Command Station](https://github.com/positron96/LocoNetControlStation).

**Warning!** This library was only tested for specific usecase and specific hardware (a ESP32 command station).
Even examples that worked for upstream library may not work with this one (I have not tested).

The core LocoNet class is now split into 2 classes: 
1. `LocoNetPhy` - an implementation of physical LocoNet bus with rx and tx functionality. 
Upon receiving a message, an Phy class sends a message to a LocoNetDispatcher for further processing.
The class is still abstract with actual implementations residing in different classes.
For ESP32, `LocoNetESP32` class was tested to work with the new architecture. 
3. `LocoNetDispatcher` - a class that broadcasts LocoNet messages to consumer objects (that implement a `LocoNetConsumer` interface). 
Has built-in filtering that was orginally present in LocoNet class.

This architecture allows for several sources of LocoNet messages to co-exist (physical LocoNet, LocoNet over WiFi, LocoNet over Serial) and to exchange messages.
Upon receiving a LocoNet message from outside world, the source classes send messages to LocoNetDispatcher, which broadcasts them to all other LocoNet sources that emit the message to outside world.

`LocoNet` class is now an alias for `LocoNetDispatcher`. Existing utility classes (`LocoNetThrottle`, `LocoNetFastClock` etc) still use `LocoNet` class and have not changed much. Probably they will work with minor modifications.
