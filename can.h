#ifndef CAN_H_
#define CAN_H_

#include <mcp_can.h>
#include "bitfield.h"
#include "serialpacket.h"
#include "carsystems.h"

static SerialPacket canInitError(0x65, 0x30);

static SerialPacket canNotInitializedError(0x65, 0x31);
static SerialPacket canTransactionError(0x65, 0x32);

static SerialPacket canSendBufferFull(0x65, 0x33);
static SerialPacket canSendTimeout(0x65, 0x34);

static SerialPacket canControlError(0x65, 0x35);

class Can {
public:
    Can(Stream * serial, uint8_t canInterruptPin, uint8_t canCsPin) {
        this->serial = serial;
        this->can = new MCP_CAN(canCsPin);
        this->canInterruptPin = canInterruptPin;
    }
    ~Can() {
        delete this->can;
        for (int i = 0; i < this->carDataCount; i++) {
            delete this->carData[i];
        }
    }
    boolean setup(uint8_t mode, uint8_t speed, uint8_t clock) {
        uint8_t canStatus = this->can->begin(mode, speed, clock);
        if (canStatus == CAN_OK) {
            this->can->setMode(MCP_NORMAL);
            pinMode(this->canInterruptPin, INPUT);
            this->isInitialized = true;
        } else {
            canInitError.serialize(&Serial);
        }
        return this->isInitialized;
    }

    void sleep() {
        this->can->setMode(MCP_SLEEP);
      }
      
    void wakeup() {
        this->can->setMode(MCP_NORMAL);
      }

    void startSniffer() {
        this->isSniffing = true;
    }

    void stopSniffer() {
        this->isSniffing = false;
    }

    bool addCanPacket(uint32_t canId, uint8_t mask) {
        if (this->carDataCount < 50) {
            this->removeCanPacket(canId);
            this->carData[this->carDataCount] = new CarData(canId, mask);
            this->carDataCount++;
            return true;
        }

        return false;
    }

    void removeCanPacket(uint32_t canId) {
        for (int index = 0; index < this->carDataCount; index++) {
            CarData * data = this->carData[index];
            if (data->getCanId() == canId) {
                for (int moveIndex = index; moveIndex < this->carDataCount - 1; moveIndex++) {
                    this->carData[moveIndex] = this->carData[moveIndex + 1];
                }
                delete data;
                this->carDataCount--;
            }
        }
    }

    void forwardFromSerial(uint8_t type, BinaryBuffer *payloadBuffer) {
        if (type != 0x62) {
            return;
        }

        BinaryData::LongResult idResult = payloadBuffer->readLong();

        if (idResult.state == BinaryData::AccessStatus::OK) {
            uint8_t data[8];
            for (uint8_t i = 0; i < payloadBuffer->getSize() - 4 && i < 8; i++) {
                BinaryData::ByteResult byteResult = payloadBuffer->readByte();
                if (byteResult.state == BinaryData::AccessStatus::OK) {
                    data[i] = byteResult.data;
                }
            }

            this->write(idResult.data, 0, payloadBuffer->getSize() - 4,
                (uint8_t*) data);
        }
    }

    void updateFromCan(void (*canCallback)(uint32_t canId, uint8_t data[], uint8_t length)) {
        if (!this->isInitialized) {
            canNotInitializedError.serialize(this->serial, 1000);
            return;
        }

        if (!digitalRead(canInterruptPin)) {
            long unsigned int canId = 0;
            uint8_t canLength = 0;
            uint8_t canData[8];

            this->can->readMsgBuf(&canId, &canLength, canData);
            if (this->isSniffing || this->carDataCount < 1) {
                this->sniff(canId, canData, canLength);
            } else {
                for (uint8_t i = 0; i < this->carDataCount; i++) {
                    if (this->carData[i]->serialize(canId, canData, this->serial)) {
                        canCallback(canId, canData, canLength);
                    }
                }
            }
        }
    }

    template<uint8_t BYTE_INDEX, uint8_t BIT_MASK, uint8_t COMPARE_VALUE>
    static bool readFlag(uint8_t * data) {
        return (data[BYTE_INDEX] & BIT_MASK) == COMPARE_VALUE;
    }
    template<uint8_t BYTE_INDEX, uint8_t BIT_MASK>
    static inline bool readFlag(uint8_t * data) {
        return Can::readFlag<BYTE_INDEX, BIT_MASK, BIT_MASK>(data);
    }

    void write(INT32U id, INT8U ext, INT8U len, INT8U *buf) {
        if (!this->isInitialized) {
            canNotInitializedError.serialize(this->serial, 1000);
            return;
        }

        uint8_t result = this->can->sendMsgBuf(id, ext, len, buf);
        if (result == CAN_GETTXBFTIMEOUT) {
            canSendBufferFull.serialize(serial, 1000);
        } else if (result == CAN_SENDMSGTIMEOUT) {
            /*
             * Happens almost every second, thus popping up errors constantly.
             * TODO: Maybe reintroduce this as a warning (no notification)?
             * canSendTimeout.serialize(serial, 1000);
             */
        }
    }
private:
    MCP_CAN * can;
    Stream * serial;
    CarData * carData[50];
    uint8_t carDataCount = 0;

    uint8_t canInterruptPin = 9;
    boolean isInitialized = false;
    boolean isSniffing = false;

    void sniff(uint32_t canId, uint8_t canData[], uint8_t length) {
        uint32_t flippedCanId = htonl(canId);
        this->serial->write("{");
        this->serial->write(0x62);
        this->serial->write(0x6d);
        this->serial->write(length + 0x04);
        this->serial->write((byte*)&flippedCanId, sizeof(canId));
        for (uint8_t i = 0; i < length; i++) {
            this->serial->write(canData[i]);
        }
        this->serial->write("}");
    }
};

#endif /* CAN_H_ */
