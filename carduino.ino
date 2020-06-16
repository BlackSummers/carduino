  /*
 Mapping Table for RTCUSTOMZ (by C. K.)

 ##PIN->PBx;PCx;PDx;ADCx/(Pin-Bezeichnung/)Pin-Nr/Digital-Pin;Analog-Pin
 ##PIN##          ##Beschreibung##        ##Code-Status##
 PB0/12/8         Standby (TJA1042T)      ready for testing
 PB1/13/9         INT (MCP2515)           ready for testing
 PB2/14/10        CS (MCP2515)            ready for testing
 PB3/(MOSI/)15/11 MOSI (MCP2515)          AUTO
 PB4/(MISO/)16/12 MISO (MCP2515)          AUTO
 PB5/(SCK/)17/13  SCK (MCP2515)           AUTO
 PB6/7            Quartz                  AUTO
 PB7/8            Quartz                  AUTO
 PC0/23/14;A0     unbelegt                fertig
 PC1/24/15;A1     EXT Input 1             NOT YET IMPLEMENTED
 PC2/25/16;A2     EXT Input 2             NOT YET IMPLEMENTED
 PC3/26/17;A3     EXT Output 1            NOT YET IMPLEMENTED
 PC4/27/18;A4     EXT Output 2            NOT YET IMPLEMENTED
 PC5/28/19;A5     unbelegt                fertig
 PC6/RESET/29     Reset (FT232RL)         externer Jumper
 PD0/30/0         RX (FT232RL)            AUTO
 PD1/31/1         TX (FT232RL)            AUTO
 PD2/32/2         Wake Input              ready for testing (no change)
 PD3/1/3          Power PD                ready for testing (no change)
 PD4/2/4          Tablet PD               ready for testing (no change)
 PD5/9/5          AMP Remote              ready for testing
 PD6/10/6         unbelegt                fertig
 PD7/11/7         unbelegt                fertig
 ADC6/19/A6       SWC1RX (Lenkrad)        ready for testing
 ADC7/22/A7       SWC2RX (Lenkrad)        ready for testing
 */

#include "Arduino.h"
#include "timer.h"
#include "network.h"
#include "carsystems.h"
#include "power.h"
#include "carduino.h"
#include "370z.h"
#include <everytime.h>

#define UNUSED(x) (void)(x)
#define ONE_SECOND 1000UL
#define ONE_MINUTE ONE_SECOND * 60


// deklariere Funktionen leer damit Carduino carduino() funktioniert
void onCarduinoSerialEvent(uint8_t type, uint8_t id, BinaryBuffer *payloadBuffer);
void onCarduinoSerialTimeout();

//CANBus Interrupt & Cs Pins
Can can(&Serial, 9, 10);

// Charger, Tablet, AMP und TJA Power Switch Pin
PowerManager powerManager(&Serial, 3, 4, 5, 8);
Carduino carduino(&Serial, onCarduinoSerialEvent, onCarduinoSerialTimeout);

//NissanClimateControl nissanClimateControl;
// ATMega Pins für Lenkradfernbedienung ADC6 und ADC7
NissanSteeringControl nissanSteeringControl(A6, A7);


Timer sleepTimer;

bool shouldSleep = false;
bool isDriverDoorOpened = true;
bool isAccessoryOn = false;

void setup() {
    carduino.begin();

    powerManager.setup();
    carduino.addCan(&can);
    carduino.addPowerManager(&powerManager);
    can.setup(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
}

// Wake Up Pin 2
void loop() {
    powerManager.update<2, RISING, ONE_SECOND / 2, ONE_MINUTE * 15>(onSleep,
            onWakeUp, onLoop);
}

void onCarduinoSerialEvent(uint8_t type, uint8_t id, BinaryBuffer *payloadBuffer) {
    UNUSED(id);
    can.forwardFromSerial(type, payloadBuffer);
    //nissanClimateControl.push(eventId, payloadBuffer);
}

void onCarduinoSerialTimeout() {
    carduino.end();
    powerManager.toggleTablet(false);
    powerManager.toggleCharger(false);
    powerManager.toggleAMP(false);
    powerManager.toggleTJA(true);
    delay(1000);
    powerManager.toggleTJA(false);
    powerManager.toggleAMP(true);
    powerManager.toggleCharger(true);
    powerManager.toggleTablet(true);
}

void onLoop() {
    if (carduino.update()) {
        can.updateFromCan(onCan);

        nissanSteeringControl.check(&carduino);

        /*
        every(250) {
            nissanClimateControl.broadcast(can);
        }
        */
    }
}

bool onSleep() {
    // Check sleep conditions
    if (isAccessoryOn) {
        // Reset timer when ACC is on to prevent premature sleep
        sleepTimer.reset();
    } else {
        // should go to sleep, when ACC is off and driver door is opened
        // Or allow operation without ACC in the closed vehicle for 30 minutes
        shouldSleep = shouldSleep || sleepTimer.check(ONE_MINUTE * 30);
    }

    if (shouldSleep) {
        carduino.end();
    }
    return shouldSleep;
}

void onWakeUp() {
    // Reset driver door status, otherwise sleep is triggered after wake up
    shouldSleep = false;
    sleepTimer.reset();
    carduino.begin();
    can.setup(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
}

void onCan(uint32_t canId, uint8_t data[], uint8_t len) {
    UNUSED(len);

    if (canId == 0x60D) {
        isAccessoryOn = Can::readFlag<1, B00000010>(data);
        bool isFrontLeftOpen = Can::readFlag<0, B00001000>(data);
        if (!isAccessoryOn && isFrontLeftOpen && !isDriverDoorOpened) {
            shouldSleep = true;
        }
        isDriverDoorOpened = isFrontLeftOpen;
    }
}
