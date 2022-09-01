#pragma once

#include "LocoNet2.h"



/************************************************************************************
 SV (System Variable Handling
 ************************************************************************************/

typedef enum {
    SV_EE_SZ_256 = 0, SV_EE_SZ_512 = 1, SV_EE_SZ_1024 = 2, SV_EE_SZ_2048 = 3, SV_EE_SZ_4096 = 4, SV_EE_SZ_8192 = 5
} SV_EE_SIZE;

typedef enum {
    SV_WRITE_SINGLE = 0x01,
    SV_READ_SINGLE = 0x02,
    SV_WRITE_MASKED = 0x03,
    SV_WRITE_QUAD = 0x05,
    SV_READ_QUAD = 0x06,
    SV_DISCOVER = 0x07,
    SV_IDENTIFY = 0x08,
    SV_CHANGE_ADDRESS = 0x09,
    SV_RECONFIGURE = 0x0F
} SV_CMD;

typedef enum {
    SV_ADDR_EEPROM_SIZE = 1,
    SV_ADDR_SW_VERSION = 2,
    SV_ADDR_NODE_ID_L = 3,
    SV_ADDR_NODE_ID_H = 4,
    SV_ADDR_SERIAL_NUMBER_L = 5,
    SV_ADDR_SERIAL_NUMBER_H = 6,
    SV_ADDR_USER_BASE = 7,
} SV_ADDR;

typedef enum {
    SV_NOT_CONSUMED = 0, SV_CONSUMED_OK = 1, SV_ERROR = 2, SV_DEFERRED_PROCESSING_NEEDED = 3
} SV_STATUS;

#define SV_MANUFACTURER_DIY		13

class LocoNetSystemVariable {
    public:
        LocoNetSystemVariable(LocoNet &locoNet, uint8_t newMfgId, uint8_t newDevId, uint16_t newProductId,
                uint8_t newSwVersion);

        /**
         * Check whether a message is an SV programming message. If so, the message
         * is processed.
         * Call this message in your main loop to implement SV programming.
         *
         * TODO: This method should be updated to reflect whether the message has
         *	been consumed.
         *
         * Note that this method will not send out replies.
         *
         * Returns:
         *		SV_OK - the message was or was not an SV programming message.
         It may or may not have been consumed.
         *		SV_DEFERRED_PROCESSING_NEEDED - the message was an SV programming
         message and has been consumed. doDeferredProcessing() must be
         called to actually process the message.
         *		SV_ERROR - the message was an SV programming message and carried
         an unsupported OPCODE.
         *
         */
        SV_STATUS processMessage(const lnMsg *LnPacket);

        /**
         * Attempts to send a reply to an SV programming message.
         * This method will repeatedly try to send the message, until it succeeds.
         *
         * Returns:
         *		SV_OK - Reply was successfully sent.
         *		SV_DEFERRED_PROCESSING_NEEDED - Reply was not sent, a later retry is needed.
         */
        SV_STATUS doDeferredProcessing(void);

        /**
         * Register SV Change callback
         *                                  SV       value    old value
         */
        void onSVChange(std::function<void(uint16_t, uint8_t, uint8_t)> callback) {
            _svChangeCallback = callback;
        }
        void reconfigureCallback(std::function<void()> callback) {
            _reconfigureCallback = callback;
        }
    private:
        LocoNet &_locoNet;
        uint8_t _mfgId;
        uint8_t _devId;
        uint16_t _productId;
        uint8_t _swVersion;
        bool _deferredProcessingRequired;
        uint8_t _deferredSrcAddr;
        std::function<void(uint16_t, uint8_t, uint8_t)> _svChangeCallback;
        std::function<void()> _reconfigureCallback;


        /** Read a value from the given EEPROM offset.
         *
         * There are two special values for the Offset parameter:
         *	SV_ADDR_EEPROM_SIZE - Return the size of the EEPROM
         *  SV_ADDR_SW_VERSION - Return the value of swVersion
         *  3 and on - Return the byte stored in the EEPROM at location (Offset - 2)
         *
         * Parameters:
         *		Offset: The offset into the EEPROM. Despite the value being passed as 2 Bytes, only the lower byte is respected.
         *
         * Returns:
         *		A Byte containing the EEPROM size, the software version or contents of the EEPROM.
         *
         */
        uint8_t readSVStorage(uint16_t Offset);

        /** Write the given value to the given Offset in EEPROM.
         *
         * TODO: Writes to Offset 0 and 1 will cause data corruption.
         *
         * Fires notifySVChanged(Offset), if the value actually chaned.
         *
         * Returns:
         *		A Byte containing the new EEPROM value (even if unchanged).
         */
        uint8_t writeSVStorage(uint16_t Offset, uint8_t Value);

        /** Checks whether the given Offset is a valid value.
         *
         * Returns:
         *		True - if the given Offset is valid. False Otherwise.
         */
        uint8_t isSVStorageValid(uint16_t Offset);

        /** Read the NodeId (Address) for SV programming of this module.
         *
         * This method accesses multiple special EEPROM locations.
         */
        uint16_t readSVNodeId(void);

        /** Write the NodeId (Address) for SV programming of this module.
         *
         * This method accesses multiple special EEPROM locations.
         */
        uint16_t writeSVNodeId(uint16_t newNodeId);

        /**
         * Checks whether all addresses of an address range are valid (defers to
         * isSVStorageValid()). Sends a notification for the first invalid address
         * (long Ack with a value of 42).
         *
         *	TODO: There is a Type error in this method. Return type is bool, but
         *		actual returned values are Integer.
         *
         * Returns:
         *		0 if at least one address of the range is not valid.
         *		1 if all addresses out of the range are valid.
         */
        bool CheckAddressRange(uint16_t startAddress, uint8_t Count);

        void reconfigure();
};



class LocoNetCV {
    public:
        //Call this method when you want to implement a module that can be configured via Uhlenbrock LNVC messages
        LocoNetCV(LocoNet &locoNet);

        /**
         * Notification that an Discover message was sent. If a module wants to react to this,
         * It should return LNCV_LACK_OK and set ArtNr and ModuleAddress accordingly.
         * A response just as in the case of ProgrammingStart will be generated.
         * If a module responds to a DiscoveryRequest, it should apparently enter programming mode immediately.
         *                                              artNr     Address
         */
        void onDiscoveryRequest(std::function<int8_t(uint16_t &, uint16_t &)> callback) {
            discoveryCallback = callback;
        }
        /**
         * Notification that a ProgrammingStart message was received. Application code should process this message and
         * set the return code to LNCV_LACK_OK in case this message was intended for this module (i.e., the addresses match).
         * In case ArtNr and/or ModuleAddress were Broadcast addresses, the Application Code should replace them by their
         * real values.
         * The calling code will then generate an appropriate ACK message.
         * A return code different than LACK_LNCV_OK will result in no response being sent.
         *                                              artNr     Address
         */
        void onProgrammingStart(std::function<int8_t(uint16_t &, uint16_t &)> callback) {
            progStartCallback = callback;
        }
        /**
         * Notification that an CV Programming Stop message was received.
         * This message is noch ACKed, thus does not require a result to be returned from the application.
         *                                          artNr     Address
         */
        void onProgrammingStop(std::function<int8_t(uint16_t, uint16_t)> callback) {
            progStopCallback = callback;
        }
        /**
         * Notification that a CV read request message was received. Application code should process this message,
         * set the cvValue (last param) to its respective value and set an appropriate return code.
         * return LNCV_LACK_OK leads the calling code to create a response containing lncvValue.
         * return code >= 0 leads to a NACK being sent.
         * return code < 0 will result in no reaction.
         *                                  artNr     CV       Return Value
         */
        void onCVRead(std::function<int8_t(uint16_t, uint16_t, uint16_t &)> callback) {
            cvReadCallback = callback;
        }
        /**
         * Notification that a CV value should be written. Application code should process this message and
         * set an appropriate return code.
         * Note 1: CV 0 is spec'd to be the ModuleAddress.
         * Note 2: Changes to CV 0 must be reflected IMMEDIATELY! E.g. the programmingStop command will
         * be sent using the new address.
         *
         * return codes >= 0 will result in a LACK containing the return code being sent.
         * return codes < 0 will result in no reaction.
         *                                  artNr     Address   CV value
         */
        void onCVWrite(std::function<int8_t(uint16_t, uint16_t, uint16_t)> callback) {
            cvWriteCallback = callback;
        }
    private:
        void makeLNCVresponse(UhlenbrockMsg & ub, uint8_t originalSource, uint16_t first, uint16_t second,
                uint16_t third, uint8_t last);
        // Computes the PXCT byte from the data bytes in the given UhlenbrockMsg.
        void computePXCTFromBytes(UhlenbrockMsg & ub);
        // Computes the correct data bytes using the containes PXCT byte
        void computeBytesFromPXCT(UhlenbrockMsg & ub);
        // Computes an address from a low- and a high-byte
        uint16_t getAddress(uint8_t lower, uint8_t higher);

        LocoNet &_locoNet;
        void processLNCVMessage(const lnMsg *LnPacket);

        std::function<int8_t(uint16_t &, uint16_t &)> discoveryCallback;
        std::function<int8_t(uint16_t &, uint16_t &)> progStartCallback;
        std::function<int8_t(uint16_t, uint16_t)> progStopCallback;
        std::function<int8_t(uint16_t, uint16_t, uint16_t &)> cvReadCallback;
        std::function<int8_t(uint16_t, uint16_t, uint16_t)> cvWriteCallback;
};
