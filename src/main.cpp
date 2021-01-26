// TODO! make nice code view
// TODO! add a few code comments

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <AccelStepper.h>
#include <MultiStepper.h>


#define LED 13
#define STEPPER1_DIR_PIN 8
#define STEPPER1_STEP_PIN 7
#define STEPPER2_DIR_PIN 10
#define STEPPER2_STEP_PIN 9

#define STEPPERS_ENABLE 4
#define ACCELERATION 200.0
#define MAX_SPEED 400.0
#define PULSE_WIDTH 1

constexpr uint32_t DEVICE_CAN_ID = 0x640;
constexpr uint32_t START_MOVING_ID = 0x080;
constexpr uint32_t ADDRESS_RECEIVE_OK_ID = 0x5C0;

constexpr uint32_t MESSAGE_TYPE = 0x23;
constexpr uint32_t SUB_REGISTER_ADDRESS = 0x00;

constexpr uint32_t CODE_SET_SPEED = 0x8160;
constexpr uint32_t CODE_SET_ACCELERATION = 0x8360;
constexpr uint32_t CODE_SET_POSITION = 0x7A60;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
MultiStepper steppers;
long stepperPositions[2];

void setup() {
    // enable LED indication
    pinMode(LED, OUTPUT);

    // enable stepper pins
    pinMode(STEPPER1_DIR_PIN, OUTPUT);
    pinMode(STEPPER1_STEP_PIN, OUTPUT);
    pinMode(STEPPER2_DIR_PIN, OUTPUT);
    pinMode(STEPPER2_STEP_PIN, OUTPUT);

    // Configure the first stepper
    stepper1.setEnablePin(STEPPERS_ENABLE);
    stepper1.setPinsInverted(false, false, true); //dir stp enable
    stepper1.setMinPulseWidth(PULSE_WIDTH);
    stepper1.setMaxSpeed(MAX_SPEED);
    stepper1.setAcceleration(ACCELERATION);

    // Configure the second stepper
    stepper2.setEnablePin(STEPPERS_ENABLE);
    stepper2.setPinsInverted(false, false, true); //dir stp enable //does not work, enabling manually
    stepper2.setMinPulseWidth(PULSE_WIDTH);
    stepper2.setMaxSpeed(MAX_SPEED);
    stepper2.setAcceleration(ACCELERATION);

    // Then give them to MultiStepper to manage
    steppers.addStepper(stepper1);
    steppers.addStepper(stepper2);


    stepperPositions[0] = 0;
    stepperPositions[1] = 0;

    // start CAN port for the CAN1 connection on the 22/23 ports
    can1.begin();
    can1.setBaudRate(500000);

    // start serial1 ports
    Serial1.begin(115200);

    // wait for serial1 port to connect. Needed for native USB port only
    while (!Serial1) { ;
    }
}

void loop() {
    if (can1.read(msg)) {
        // print to serial, just for debug
        Serial1.print("CAN1 ");
        Serial1.print("MB: ");
        Serial1.print(msg.mb, HEX);
        Serial1.print("  ID: 0x");
        Serial1.print(msg.id, HEX);
        Serial1.print("  EXT: ");
        Serial1.print(msg.flags.extended, HEX);
        Serial1.print("  LEN: ");
        Serial1.print(msg.len, HEX);
        Serial1.print(" DATA: ");
        for (uint8_t i = 0; i < 8; i++) {
            Serial1.print(msg.buf[i], HEX);
            Serial1.print(" ");
        }
        Serial1.print("  TS: ");
        Serial1.println(msg.timestamp);

        if (DEVICE_CAN_ID == msg.id) {
            // send back received message, which confirm message getting.
            msg.id = ADDRESS_RECEIVE_OK_ID;
            can1.write(msg);

            // get message code type
            uint32_t messageCode = msg.buf[1] << 8;
            messageCode |= msg.buf[2];

            // get message data from code
            uint32_t messageData = msg.buf[7] << 24;
            messageData |= msg.buf[6] << 16;
            messageData |= msg.buf[5] << 8;
            messageData |= msg.buf[4];

            // print to serial a few debug messages
            Serial1.print("messageCode: ");
            Serial1.println(messageCode, HEX);
            Serial1.print("messageData: ");
            Serial1.print(messageData, HEX);
            Serial1.println();

            switch (messageCode) {
                case CODE_SET_SPEED:
                    stepper1.setMaxSpeed(messageData);
                    stepper2.setMaxSpeed(messageData);
                    Serial1.println("MaxSpeed is set");
                    break;
                case CODE_SET_ACCELERATION:
                    stepper1.setAcceleration(messageData);
                    stepper2.setAcceleration(messageData);
                    Serial1.println("Acceleration is set");
                    break;
                case CODE_SET_POSITION:
                    stepperPositions[0] = messageData;
                    stepperPositions[1] = messageData;
                    Serial1.println("Position is set");
                    break;
            }

        } else if (START_MOVING_ID == msg.id) {
            Serial1.println("Moving is START");
            stepper1.enableOutputs();
            stepper2.enableOutputs();
            steppers.moveTo(stepperPositions);
            bool isMoved = false;
            while (steppers.run()) {
                isMoved = true;
            }
            stepper1.disableOutputs();
            stepper2.disableOutputs();
            msg.id = ADDRESS_RECEIVE_OK_ID;
            for (uint8_t i = 0; i < 8; i++) {
                msg.buf[i] = 0x0;
            }
            if (isMoved) {
                Serial1.println("Moving is DONE");
                can1.write(msg);
            } else {
                Serial1.println("Moving is NOT HAPPEN");
            }
        }

    }



    //Blink LED
    int time = round(millis() / 2000.0);
    if (time % 2 == 0) {
        digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    } else {
        digitalWrite(LED, LOW);   // turn the LED on (HIGH is the voltage level)
    }
}