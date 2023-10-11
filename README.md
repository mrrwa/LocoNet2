# LocoNet2
A refactor of the original [LocoNet](https://github.com/mrrwa/LocoNet) library to be much more object oriented, with derived classes for each different hardware interface

Currently, the library supports the ESP32 and RPi Pico (RP2040) micro controllers.

This version of the LocoNet2 library has built on the code contributions from [positron96/LocoNet2](https://github.com/positron96/LocoNet2)

The core LocoNet class is now split into 2 classes: 
1. `LocoNetPhy` - an implementation of physical LocoNet bus with rx and tx functionality. 
Upon receiving a message, an Phy class sends a message to a LocoNetDispatcher for further processing.

The class is still abstract with actual implementations residing in different classes.
For ESP32,  `LocoNetESP32` class was tested to work with the new architecture.
For RP2040, `LocoNetRP2040` class was tested to work with the new architecture.
 
2. `LocoNetDispatcher` - a class that broadcasts LocoNet messages to consumer objects (that implement a `LocoNetConsumer` interface). 
Has built-in filtering that was orginally present in LocoNet class.

This architecture allows for several sources of LocoNet messages to co-exist (physical LocoNet, LocoNet over WiFi, LocoNet over Serial) and to exchange messages.
Upon receiving a LocoNet message from outside world, the source classes send messages to LocoNetDispatcher, which broadcasts them to all other LocoNet sources that emit the message to outside world.

`LocoNet` class is now an alias for `LocoNetDispatcher`. Existing utility classes (`LocoNetThrottle`, `LocoNetFastClock` etc) still use `LocoNet` class and have not changed much. Probably they will work with minor modifications.
