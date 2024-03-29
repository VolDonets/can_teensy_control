// TODO! make nice code view
// TODO! add a few code comments

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Adafruit_ADS1015.h>
#include <Bounce2.h>
#include <TeensyThreads.h>
#include <PID_v1.h>

/// BOARD ID value (it determines an board address in CAN bus)
constexpr int8_t BOARD_ID = 0x0;

//------ STOPS ------
#define STOP_BOT_PIN      11
#define STOP_BOT_DEBOUNCE 30 //ms
#define STOP_TOP_PIN      12
#define STOP_TOP_DEBOUNCE 30 //ms

#define LED 13
#define STEPPER1_DIR_PIN 8
#define STEPPER1_STEP_PIN 7
#define STEPPER2_DIR_PIN 10
#define STEPPER2_STEP_PIN 9

#define STEPPERS_ENABLE 4
#define ACCELERATION 200.0
#define MAX_SPEED 400.0
#define PULSE_WIDTH 1

#define ADS1115_I2C_ADDRESS_GND 0x48
#define ADS1115_I2C_ADDRESS_VDD 0x49

#define PWM_PIN0 14
#define PWM_PIN1 15
#define PWM_PIN2 2
#define PWM_PIN3 3

#define PIN_FAN1 2
#define PIN_FAN2 3

//#define ADC_CS1_ALERT_PIN 16
//#define ADC_CS2_ALERT_PIN 17

constexpr uint32_t DEVICE_CAN_ID = 0x640 + BOARD_ID;
constexpr uint32_t START_MOVING_ID = 0x080;
constexpr uint32_t ADDRESS_RECEIVE_OK_ID = 0x5C0 + BOARD_ID;
constexpr uint32_t VELOCITY_POSITION_STEPPER_0_DEV_ID = 0x440;
constexpr uint32_t VELOCITY_POSITION_STEPPER_1_DEV_ID = 0x441;
constexpr uint32_t VELOCITY_POSITION_OK_ID = 0x3C0;
constexpr uint32_t HOMING_RESULT_CAN_ID = 0x3C0;

constexpr uint32_t MESSAGE_TYPE_SET = 0x23;
constexpr uint32_t MESSAGE_TYPE_REQUEST = 0x40;
constexpr uint32_t MESSAGE_TYPE_REPORT = 0x43;

constexpr uint32_t CODE_SET_SPEED = 0x8160;
constexpr uint32_t CODE_SET_ACCELERATION = 0x8360;
constexpr uint32_t CODE_SET_POSITION = 0x7A60;
constexpr uint32_t CODE_SET_HOME_OFFSET = 0x7C60;
constexpr uint32_t CODE_SET_HOME_SPEED = 0x9960;
constexpr uint32_t CODE_SET_GO_HOME = 0x6060;
constexpr uint32_t CODE_SET_HOT_END_TEMPERATURE = 0x0063;
constexpr uint32_t CODE_GET_REPORT_TEMPERATURE = 0x0163;
constexpr uint32_t CODE_SET_FAN_SPEED = 0x8083;

constexpr uint32_t MESSAGE_TYPE_REQUEST_CHANNEL_AD_DATA = 0x40;
constexpr uint32_t MESSAGE_TYPE_RETURN_CHANNEL_AD_DATA = 0x43;
constexpr uint32_t CODE_CHANNEL_AD = 0x0061;

constexpr uint32_t MESSAGE_TYPE_SET_PWM = 0x23;
constexpr uint32_t CODE_SET_PWM = 0x0062;

constexpr uint32_t RESOLUTION_PWM_BYTE = 2;
constexpr float RESOLUTION_PWM_BIT = 32767.0;

constexpr uint32_t CODE_GET_ENDSTOP_STATES = 0xFD60;


constexpr uint8_t SET_MODE_CODE_SIMPLE_MODE = 0x00;
constexpr uint8_t SET_MODE_CODE_HOMING = 0x01;
constexpr uint8_t SET_MODE_CODE_SINGLE_Z_PROBE = 0x02;
constexpr uint8_t SET_MODE_CODE_STOP_MOTION = 0x03;
constexpr uint8_t SET_MODE_CODE_DISABLE_STEPPER = 0x10;
constexpr uint8_t SET_MODE_CODE_ENABLE_STEPPER = 0x11;

constexpr uint32_t CODE_SET_GET_PID_P = 0x5F5A;
constexpr uint32_t CODE_SET_GET_PID_I = 0x5F5B;
constexpr uint32_t CODE_SET_GET_PID_D = 0x5F5C;

/// this constant are multiplicate (when send it) PID params or divide (when recive it)
constexpr double MULT_PID_CONST = 1000.0;

constexpr uint32_t CODE_SET_PIN_MODE_INPUT = 0x3E3A;
constexpr uint32_t CODE_SET_PIN_MODE_OUTPUT = 0x3E3B;
constexpr uint32_t CODE_SET_PIN_MODE_INPUT_PULLUP = 0x3E3C;
constexpr uint32_t CODE_SET_PIN_MODE_INPUT_PULLDOWN = 0x3E3D;

// // ADC Range: +/- 6.144V (1 bit = 3mV)
// // EXAMPLE Comparator for ADC_CS1 Threshold: 1000 (3.000V)
// // Comparator for ADC_CS1 Threshold: 166 (0.500V)
//constexpr int16_t ADC_CS1_THRESHOLD_LEVEL = 17600;
// // EXAMPLE Comparator for ADC_CS2 Threshold: 1000 (3.000V)
// // Comparator for ADC_CS2 Threshold: 166 (0.500V)
//constexpr int16_t ADC_CS2_THRESHOLD_LEVEL = 17600;


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;
volatile bool isCANMessageAvailable;

AccelStepper stepper0(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
MultiStepper steppers;
long stepperPositions[2];
Adafruit_ADS1115 adsGND(ADS1115_I2C_ADDRESS_GND);
Adafruit_ADS1115 adsVDD(ADS1115_I2C_ADDRESS_VDD);

float justMovingSpeedStepper1 = 0.0;
float justMovingSpeedStepper2 = 0.0;

long homePositionOffset = 0;
float homeMovingSpeed = 200.0;

bool isNotStoppedByCAN = true;
bool isTopNotStopped;

float hotEndTempHeater0;
float hotEndTempHeater1;

bool isNewPositionForStepper0 = false;
bool isNewPositionForStepper1 = false;

/// this is a debouncer, which prevent mechanical noise on stop-end activation
Bounce debouncerTop = Bounce();
/// this is a debouncer, which prevent mechanical noise on stop-end activation
Bounce debouncerBottom = Bounce();

/// PID controller for the heater0 and it's params
double setpointPIDHeater0;
double inputPIDHeater0;
double outputPIDHeater0;
double kpPID0 = 2.0, kiPID0 = 5.0, kdPID0 = 1;
PID heaterPID0(&inputPIDHeater0, &outputPIDHeater0, &setpointPIDHeater0, kpPID0, kiPID0, kdPID0, DIRECT);

/// PID controller for the heater1 and it's params
double setpointPIDHeater1;
double inputPIDHeater1;
double outputPIDHeater1;
double kpPID1 = 2.0, kiPID1 = 5.0, kdPID1 = 1;
PID heaterPID1(&inputPIDHeater1, &outputPIDHeater1, &setpointPIDHeater1, kpPID1, kiPID1, kdPID1, DIRECT);

void stopTopContactISR() {
    debouncerTop.update();
    // Call code if button transitions from HIGH to LOW
    if (debouncerTop.fell()) {
        Serial1.println("fell() = TRUE");
    } else {
        Serial1.println("fell() = FALSE");
    }
    isTopNotStopped = false;
}

void stopBottomContactISR() {

}

//void onLeverExceedingADC_CS1() {
//    Serial1.println("Level is exceeding for ADC CS_1");
//}
//
//void onLeverExceedingADC_CS2() {
//    Serial1.println("Level is exceeding for ADC CS_2");
//}

void processHoming() {
    // disable CAN stopping;
    isNotStoppedByCAN = true;

    // set homeMovingSpeed as a maximum speed for the steppers
    stepper0.setMaxSpeed(homeMovingSpeed);
    stepper1.setMaxSpeed(homeMovingSpeed);
    stepper0.enableOutputs();
    stepper1.enableOutputs();

    // move while no stop flag enabled, for this look function stopTopContactISR()
    isTopNotStopped = true;
    stepperPositions[0] = stepper0.currentPosition() + 100;
    stepperPositions[1] = stepper1.currentPosition() + 100;
    steppers.moveTo(stepperPositions);
    while (isNotStoppedByCAN && isTopNotStopped && steppers.run()) {
        if (abs(stepper0.targetPosition() - stepper0.currentPosition()) < 50) {
            stepperPositions[0] = stepper0.currentPosition() + 100;
            stepperPositions[1] = stepper1.currentPosition() + 100;
            steppers.moveTo(stepperPositions);
        }
    }


    // move while no stop flag disable, for this look function stopTopContactISR()
    isTopNotStopped = true;
    stepperPositions[0] = stepper0.currentPosition() - 100;
    stepperPositions[1] = stepper1.currentPosition() - 100;
    steppers.moveTo(stepperPositions);
    while (isNotStoppedByCAN && isTopNotStopped && steppers.run()) {
        if (abs(stepper0.targetPosition() - stepper0.currentPosition()) < 50) {
            stepperPositions[0] = stepper0.currentPosition() - 100;
            stepperPositions[1] = stepper1.currentPosition() - 100;
            steppers.moveTo(stepperPositions);
        }
    }

    // move from the stop point on the offset
    stepperPositions[0] = stepper0.currentPosition() - homePositionOffset;
    stepperPositions[1] = stepper1.currentPosition() - homePositionOffset;
    steppers.moveTo(stepperPositions);
    while (isNotStoppedByCAN && steppers.run()) { ; }
//    stepper0.disableOutputs();
//    stepper1.disableOutputs();
}

void processSimpleMoving() {
    Serial1.println("Moving is START");


    // disable CAN stopping;
    isNotStoppedByCAN = true;

    stepper0.setMaxSpeed(justMovingSpeedStepper1);
    stepper1.setMaxSpeed(justMovingSpeedStepper2);

    stepper0.enableOutputs();
    stepper1.enableOutputs();
    steppers.moveTo(stepperPositions);
    bool isMoved = false;
    if (isNewPositionForStepper0 && isNewPositionForStepper1) {
        while (isNotStoppedByCAN && steppers.run()) {
            isMoved = true;
        }
    }
//                stepper0.disableOutputs();
//                stepper1.disableOutputs();
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

    isNewPositionForStepper0 = false;
    isNewPositionForStepper1 = false;
}

void sendCurrentSteppersPosition(CAN_message_t &msgCAN) {
    // return a new current position for the master device
    msgCAN.id = HOMING_RESULT_CAN_ID;
    // set position
    msgCAN.buf[0] = stepper0.currentPosition();
    msgCAN.buf[1] = stepper0.currentPosition() >> 8;
    msgCAN.buf[2] = stepper0.currentPosition() >> 16;
    msgCAN.buf[3] = stepper0.currentPosition() >> 24;
    // set speed
    msgCAN.buf[4] = (int32_t) homeMovingSpeed;
    msgCAN.buf[5] = ((int32_t) homeMovingSpeed) >> 8;
    msgCAN.buf[6] = ((int32_t) homeMovingSpeed) >> 16;
    msgCAN.buf[7] = ((int32_t) homeMovingSpeed) >> 24;
    // send a message to the CAN1
    can1.write(msgCAN);
}

[[noreturn]]
void messageParseCANByThread() {
    while (true) {
        if (isCANMessageAvailable) {
            Serial1.println("Hello from thread");
            if (DEVICE_CAN_ID == msg.id) {
                // get message code type
                uint32_t messageCode = msg.buf[1] << 8;
                messageCode |= msg.buf[2];

                switch (messageCode) {
                    case CODE_SET_GO_HOME:
                        switch (msg.buf[4]) {
                            case SET_MODE_CODE_HOMING:
                                processHoming();
                                sendCurrentSteppersPosition(msg);

                                // a bit debug
                                Serial1.println("Homing is DONE!");
                                break;
                        }
                        break;
                }
            } else if (START_MOVING_ID == msg.id) {
                processSimpleMoving();
            }

            isCANMessageAvailable = false;
        }
    }
}

[[noreturn]]
void heater0PIDControllerThread() {
    while (true) {
        inputPIDHeater0 = adsVDD.readADC_Differential_0_1();
        heaterPID0.Compute();
        analogWrite(PWM_PIN0, outputPIDHeater0);
    }
}

void copyCANMessage(const CAN_message_t &msgCAN) {
    msg.id = msgCAN.id;
    for (uint8_t i = 0; i < 8; i++) {
        msg.buf[i] = msgCAN.buf[i];
    }
}

double parseParamPID(const uint32_t &messageData) {
    int32_t signedMessageData = messageData;
    return ((double) signedMessageData) / MULT_PID_CONST;
}

void prepareResponseMessageParamPID(CAN_message_t &msgSend, const double &pidCoef) {
    double dataForSendingDouble = pidCoef * MULT_PID_CONST;
    int32_t dataForSendingInt = dataForSendingDouble;
    uint32_t dataForSending = dataForSendingInt;
    msgSend.buf[0] = MESSAGE_TYPE_REPORT;
    msgSend.buf[4] = dataForSending;
    msgSend.buf[5] = dataForSending >> 8;
    msgSend.buf[6] = dataForSending >> 16;
    msgSend.buf[7] = dataForSending >> 24;
}

void canMessagesSniff(const CAN_message_t &msgCAN) {
    // skip sync messages when id doesn't need
    if ((msgCAN.id == START_MOVING_ID) && (!(isNewPositionForStepper0 && isNewPositionForStepper1)))
        return;

    CAN_message_t msgSend;

    if (msgCAN.id != 0x080) {
        Serial1.print("CAN1 ");
        Serial1.print("MB: ");
        Serial1.print(msgCAN.mb, HEX);
        Serial1.print("  ID: 0x");
        Serial1.print(msgCAN.id, HEX);
        Serial1.print("  EXT: ");
        Serial1.print(msgCAN.flags.extended, HEX);
        Serial1.print("  LEN: ");
        Serial1.print(msgCAN.len, HEX);
        Serial1.print(" DATA: ");
    }
    msgSend.id = msgCAN.id;
    for (uint8_t i = 0; i < 8; i++) {
        if (msgCAN.id != 0x080) {
            Serial1.print(msgCAN.buf[i], HEX);
            Serial1.print(" ");
        }
        msgSend.buf[i] = msgCAN.buf[i];
    }
    if (msgCAN.id != 0x080) {
        Serial1.print("  TS: ");
        Serial1.println(msgCAN.timestamp);
    }


    if ((VELOCITY_POSITION_STEPPER_0_DEV_ID == msgCAN.id)
        || (VELOCITY_POSITION_STEPPER_1_DEV_ID == msgCAN.id)) {
        // get message data (position) from code
        uint32_t messageDataPosition = msgCAN.buf[3] << 24;
        messageDataPosition |= msgCAN.buf[2] << 16;
        messageDataPosition |= msgCAN.buf[1] << 8;
        messageDataPosition |= msgCAN.buf[0];

        // get message data (speed=velocity) from code
        uint32_t messageDataVelocity = msgCAN.buf[7] << 24;
        messageDataVelocity |= msgCAN.buf[6] << 16;
        messageDataVelocity |= msgCAN.buf[5] << 8;
        messageDataVelocity |= msgCAN.buf[4];

        // set max speed to steppers and set position to steppers
        switch (msgCAN.id) {
            case VELOCITY_POSITION_STEPPER_0_DEV_ID:
                justMovingSpeedStepper1 = messageDataVelocity;
                stepperPositions[0] = messageDataPosition;
                isNewPositionForStepper0 = true;

                // a bit debug
                Serial1.print("MaxSpeed is set for stepper_0: ");
                Serial1.println(justMovingSpeedStepper1);
                Serial1.print("Position is set for stepper_0: ");
                Serial1.println(stepperPositions[0]);
                break;
            case VELOCITY_POSITION_STEPPER_1_DEV_ID:
                justMovingSpeedStepper2 = messageDataVelocity;
                stepperPositions[1] = messageDataPosition;
                isNewPositionForStepper1 = true;

                // a bit debug
                Serial1.print("MaxSpeed is set for stepper_1: ");
                Serial1.println(justMovingSpeedStepper2);
                Serial1.print("Position is set for stepper_1: ");
                Serial1.println(stepperPositions[1]);
                break;
        }

        // send back received message, which confirm message getting.
        msgSend.id = VELOCITY_POSITION_OK_ID;
        can1.write(msgSend);
    } else if (DEVICE_CAN_ID == msgCAN.id) {
        // send back received message, which confirm message getting.
        msgSend.id = ADDRESS_RECEIVE_OK_ID;

        // get message code type
        uint32_t messageCode = msgCAN.buf[1] << 8;
        messageCode |= msgCAN.buf[2];

        // get message data from code
        uint32_t messageData = msgCAN.buf[7] << 24;
        messageData |= msgCAN.buf[6] << 16;
        messageData |= msgCAN.buf[5] << 8;
        messageData |= msgCAN.buf[4];

        // print to serial a few debug messages
        Serial1.print("messageCode: ");
        Serial1.println(messageCode, HEX);
        Serial1.print("messageData: ");
        Serial1.print(messageData, HEX);
        Serial1.println();

        int16_t adData = 0;

        float pwmValue;

        switch (messageCode) {
            case CODE_SET_SPEED:
                justMovingSpeedStepper1 = messageData;
                justMovingSpeedStepper2 = messageData;

                // send confirmation message back by CAN
                can1.write(msgSend);

                // a bit debug
                Serial1.println("MaxSpeed is set");
                break;
            case CODE_SET_ACCELERATION:
                stepper0.setAcceleration(messageData);
                stepper1.setAcceleration(messageData);

                // send confirmation message back by CAN
                can1.write(msgSend);

                // a bit debug
                Serial1.println("Acceleration is set");
                break;
            case CODE_SET_POSITION:
                stepperPositions[0] = messageData;
                stepperPositions[1] = messageData;
                isNewPositionForStepper0 = true;
                isNewPositionForStepper1 = true;

                // send confirmation message back by CAN
                can1.write(msgSend);

                // a bit debug
                Serial1.println("Position is set");
                break;
            case CODE_CHANNEL_AD:
                // read data from interested sensor
                switch (msgCAN.buf[3]) {
                    case 0x00:
                        adData = adsVDD.readADC_Differential_0_1();
                        break;
                    case 0x01:
                        adData = adsVDD.readADC_Differential_2_3();
                        break;
                    case 0x02:
                        adData = adsGND.readADC_Differential_0_1();
                        break;
                    case 0x03:
                        adData = adsGND.readADC_Differential_2_3();
                        break;
                }

                // send confirmation message back by CAN
                msgSend.buf[0] = MESSAGE_TYPE_RETURN_CHANNEL_AD_DATA;
                msgSend.buf[4] = adData;
                msgSend.buf[5] = adData >> 8;
                msgSend.buf[6] = 0x0;
                msgSend.buf[7] = 0x0;
                can1.write(msgSend);

                // a bit debug messages to Serial1
                Serial1.print("get AM");
                Serial1.print(msgSend.buf[3], HEX);
                Serial1.print(": ");
                Serial1.println(adData);
                break;
            case CODE_SET_PWM:
                pwmValue = messageData;
                pwmValue = (pwmValue * RESOLUTION_PWM_BIT) / 100;
                switch (msgCAN.buf[3]) {
                    case 0x00:
                        analogWrite(PWM_PIN0, pwmValue);
                        break;
                    case 0x01:
                        analogWrite(PWM_PIN1, pwmValue);
                        break;
                    case 0x02:
                        analogWrite(PWM_PIN2, pwmValue);
                        break;
                    case 0x03:
                        analogWrite(PWM_PIN3, pwmValue);
                        break;
                }

                // send confirmation message back by CAN
                can1.write(msgSend);

                // a bit debug messages to Serial1
                Serial1.print("set PWM for PIN:");
                Serial1.println();
                break;
            case CODE_SET_HOME_OFFSET:
                homePositionOffset = messageData;

                // send confirmation message back by CAN
                can1.write(msgSend);

                // a bit debug messages to Serial1
                Serial1.print("Offset near home position: ");
                Serial1.println(homePositionOffset, DEC);
                break;
            case CODE_SET_HOME_SPEED:
                homeMovingSpeed = messageData;

                // send confirmation message back by CAN
                can1.write(msgSend);

                // a bit debug messages to Serial1
                Serial1.print("Moving speed to home: ");
                Serial1.println(homeMovingSpeed, DEC);
                break;
            case CODE_SET_GO_HOME:
                // get data from message
                switch (msgCAN.buf[4]) {
                    case SET_MODE_CODE_SIMPLE_MODE:

                        // send message with data
                        sendCurrentSteppersPosition(msgSend);
                        break;
                    case SET_MODE_CODE_HOMING:
                        // send for processing in other thread
                        copyCANMessage(msgCAN);
                        isCANMessageAvailable = true;
                        // a bit debug
                        Serial1.println("Homing will be processed in the other thread");
                        break;
                    case SET_MODE_CODE_SINGLE_Z_PROBE:
                        // TODO in the future here should be a code

                        // send confirmation message back by CAN
                        can1.write(msgSend);

                        break;
                    case SET_MODE_CODE_STOP_MOTION:
                        isNotStoppedByCAN = false;
                        // send confirmation message back by CAN
                        can1.write(msgSend);
                        // an answer will be after ending moving
                        break;
                    case SET_MODE_CODE_DISABLE_STEPPER:
                        stepper0.disableOutputs();
                        stepper1.disableOutputs();

                        // send message with data
                        sendCurrentSteppersPosition(msgSend);

                        // a bit debug messages:
                        Serial1.print("steppers are DISABLED");
                        break;
                    case SET_MODE_CODE_ENABLE_STEPPER:
                        stepper0.enableOutputs();
                        stepper1.enableOutputs();

                        // send message with data
                        sendCurrentSteppersPosition(msgSend);

                        // a bit debug messages:
                        Serial1.print("steppers are ENABLED");
                        break;
                }
                break;
            case CODE_GET_ENDSTOP_STATES:
                // prepare a report message and read current PINs states
                // and send message with data by CAN
                msgSend.id = ADDRESS_RECEIVE_OK_ID;
                msgSend.buf[0] = MESSAGE_TYPE_REPORT;
                msgSend.buf[3] = 0x00;
                msgSend.buf[4] = 0x00;
                msgSend.buf[5] = 0x00;
                msgSend.buf[6] = (digitalRead(STOP_BOT_PIN) << 1) + digitalRead(STOP_TOP_PIN);
                msgSend.buf[7] = 0x00;
                can1.write(msgSend);

                // a bit debug
                Serial1.print("Send values for the pin_11: ");
                Serial1.print(digitalRead(STOP_BOT_PIN), DEC);
                Serial1.print(", for the pin_12: ");
                Serial1.println(digitalRead(STOP_TOP_PIN));
                break;

            case CODE_SET_HOT_END_TEMPERATURE:
                //get by channel get and set aim temperature
                switch (msgCAN.buf[3]) {
                    case 0x00: // the 0s channel
                        hotEndTempHeater0 = messageData;
                        break;
                    case 0x01: // the 1st channel
                        hotEndTempHeater1 = messageData;
                        break;
                }

                // send confirmation message back by CAN
                can1.write(msgSend);

                // a bit debug
                Serial1.print("Set HOT_END_TEMP for heater");
                Serial1.print(msgCAN.buf[3], HEX);
                Serial1.print(": temp=");
                switch (msgCAN.buf[3]) {
                    case 0x00: // the 0s channel
                        Serial1.println(hotEndTempHeater0, DEC);
                        break;
                    case 0x01: // the 1st channel
                        Serial1.println(hotEndTempHeater1, DEC);
                        break;
                }
                break;
            case CODE_GET_REPORT_TEMPERATURE:
                //get by channel and return current temperature
                // TODO need to fix temperature source sensor
                // TODO need to parse data from sensors
                switch (msgCAN.buf[3]) {
                    case 0x00: // the 0s channel
                        adData = adsVDD.readADC_Differential_0_1();
                        break;
                    case 0x01: // the 1st channel
                        adData = adsVDD.readADC_Differential_2_3();
                        break;
                }

                // send confirmation message back by CAN
                msgSend.id = ADDRESS_RECEIVE_OK_ID;
                msgSend.buf[0] = MESSAGE_TYPE_RETURN_CHANNEL_AD_DATA;
                msgSend.buf[4] = adData;
                msgSend.buf[5] = adData >> 8;
                msgSend.buf[6] = 0x0;
                msgSend.buf[7] = 0x0;
                can1.write(msgSend);

                // a bit debug
                Serial1.print("Read temperature for sensor");
                Serial1.print(msgCAN.buf[3], HEX);
                Serial1.print(": tmp=");
                Serial1.println(adData);
                break;

            case CODE_SET_FAN_SPEED:
                // warning fan speed sets from 0 to 255
                pwmValue = ((float) messageData) / 255.0;
                pwmValue = (pwmValue * RESOLUTION_PWM_BIT);
                switch (msgCAN.buf[3]) {
                    case 0x00: // setting fan speed for the fan1
                        analogWrite(PIN_FAN1, pwmValue);
                        break;
                    case 0x01: // setting fan speed for the fan2
                        analogWrite(PIN_FAN2, pwmValue);
                        break;
                }

                // send confirmation message back by CAN
                can1.write(msgSend);

                // a bit debug
                Serial1.print("Set speed for FAN:");
                Serial1.print(msgCAN.buf[3] + 1, HEX);
                Serial1.print(": val=");
                Serial1.println(pwmValue);
                break;
            case CODE_SET_GET_PID_P:
                switch (msgCAN.buf[0]) {
                    case MESSAGE_TYPE_SET:
                        // set values
                        switch (msgCAN.buf[3]) {
                            case 0x00: // set P value for PID of heater 0
                                kpPID0 = parseParamPID(messageData);

                                // a bit debug messages to serial
                                Serial1.print("set heater_0 kP=");
                                Serial1.println(kpPID0);
                                break;
                            case 0x01: // set P value for PID of heater 1
                                kpPID1 = parseParamPID(messageData);

                                // a bit debug messages to serial
                                Serial1.print("set heater_1 kP=");
                                Serial1.println(kpPID1);
                                break;
                        }

                        // sent confirmation message via CAN
                        can1.write(msgSend);
                        break;
                    case MESSAGE_TYPE_REQUEST:
                        switch (msgCAN.buf[3]) {
                            case 0x00: // get P value for PID of heater 0
                                prepareResponseMessageParamPID(msgSend, kpPID0);
                                break;
                            case 0x01: // get P value for PID of heater 1
                                prepareResponseMessageParamPID(msgSend, kpPID1);
                                break;
                        }

                        // sent confirmation message via CAN with data
                        can1.write(msgSend);

                        // a bit debug messages to serial
                        Serial1.print("Send P value heater_");
                        Serial1.println(msgCAN.buf[3]);
                        break;
                }
                break;
            case CODE_SET_GET_PID_I:
                switch (msgCAN.buf[0]) {
                    case MESSAGE_TYPE_SET:
                        // set values
                        switch (msgCAN.buf[3]) {
                            case 0x00: // set I value for PID of heater 0
                                kiPID0 = parseParamPID(messageData);

                                // a bit debug messages to serial
                                Serial1.print("set heater_0 kI=");
                                Serial1.println(kiPID0);
                                break;
                            case 0x01: // set I value for PID of heater 1
                                kiPID1 = parseParamPID(messageData);

                                // a bit debug messages to serial
                                Serial1.print("set heater_1 kI=");
                                Serial1.println(kiPID1);
                                break;
                        }

                        // sent confirmation message via CAN
                        can1.write(msgSend);
                        break;
                    case MESSAGE_TYPE_REQUEST:
                        switch (msgCAN.buf[3]) {
                            case 0x00: // get I value for PID of heater 0
                                prepareResponseMessageParamPID(msgSend, kiPID0);
                                break;
                            case 0x01: // get I value for PID of heater 1
                                prepareResponseMessageParamPID(msgSend, kiPID1);
                                break;
                        }

                        // sent confirmation message via CAN with data
                        can1.write(msgSend);

                        // a bit debug messages to serial
                        Serial1.print("Send I value heater_");
                        Serial1.println(msgCAN.buf[3]);
                        break;
                }
                break;
            case CODE_SET_GET_PID_D:
                switch (msgCAN.buf[0]) {
                    case MESSAGE_TYPE_SET:
                        // set values
                        switch (msgCAN.buf[3]) {
                            case 0x00: // set D value for PID of heater 0
                                kdPID0 = parseParamPID(messageData);

                                // a bit debug messages to serial
                                Serial1.print("set heater_0 kD=");
                                Serial1.println(kdPID0);
                                break;
                            case 0x01: // set D value for PID of heater 1
                                kdPID1 = parseParamPID(messageData);

                                // a bit debug messages to serial
                                Serial1.print("set heater_1 kD=");
                                Serial1.println(kdPID1);
                                break;
                        }

                        // sent confirmation message via CAN
                        can1.write(msgSend);
                        break;
                    case MESSAGE_TYPE_REQUEST:
                        switch (msgCAN.buf[3]) {
                            case 0x00: // get D value for PID of heater 0
                                prepareResponseMessageParamPID(msgSend, kdPID0);
                                break;
                            case 0x01: // get D value for PID of heater 1
                                prepareResponseMessageParamPID(msgSend, kdPID1);
                                break;
                        }

                        // sent confirmation message via CAN with data
                        can1.write(msgSend);

                        // a bit debug messages to serial
                        Serial1.print("Send D value heater_");
                        Serial1.println(msgCAN.buf[3]);
                        break;
                }
                break;
            case CODE_SET_PIN_MODE_INPUT:
                pinMode(msgCAN.buf[3], INPUT);

                // a bit debug
                Serial1.print("INPUT mode for PIN_");
                Serial1.print(msgCAN.buf[3]);
                Serial1.println(" is ENABLED");
                break;
            case CODE_SET_PIN_MODE_OUTPUT:
                pinMode(msgCAN.buf[3], OUTPUT);
                // make PWM control only if messageData != 0;
                if (messageData) {
                    // warning fan speed sets from 1 to 255
                    pwmValue = ((float) messageData) / 255.0;
                    pwmValue = (pwmValue * RESOLUTION_PWM_BIT);

                    analogWriteFrequency(msgCAN.buf[3], 1000);
                    analogWriteResolution(15);
                    analogWrite(msgCAN.buf[3], 0);
                }

                // a bit debug
                Serial1.print("OUTPUT mode for PIN_");
                Serial1.print(msgCAN.buf[3]);
                Serial1.println(" is ENABLED");
                if (messageData) {
                    Serial1.print("Also PIN_");
                    Serial1.print(msgCAN.buf[3]);
                    Serial1.print(": PWM = ");
                    Serial1.print(pwmValue);
                    Serial1.print(" / ");
                    Serial1.println(RESOLUTION_PWM_BIT);
                }
                break;
            case CODE_SET_PIN_MODE_INPUT_PULLUP:
                pinMode(msgCAN.buf[3], INPUT_PULLUP);

                // a bit debug
                Serial1.print("INPUT_PULLUP mode for PIN_");
                Serial1.print(msgCAN.buf[3]);
                Serial1.println(" is ENABLED");
                break;
            case CODE_SET_PIN_MODE_INPUT_PULLDOWN:
                pinMode(msgCAN.buf[3], INPUT_PULLDOWN);

                // a bit debug
                Serial1.print("INPUT_PULLDOWN mode for PIN_");
                Serial1.print(msgCAN.buf[3]);
                Serial1.println(" is ENABLED");
                break;
        }

    } else if (START_MOVING_ID == msgCAN.id) {
        // send for processing it in other thread
        if (isNewPositionForStepper0 && isNewPositionForStepper1) {
            copyCANMessage(msgCAN);
            isCANMessageAvailable = true;
        }
    }

}

//[[noreturn]]
//void leverPrinterThread() {
//    while (true) {
//        Serial1.print("adsVDD: ");
// //        Serial1.print("sg_0=");
// //        Serial1.print(adsVDD.readADC_SingleEnded(0), DEC);
//        Serial1.print(", sg_1=");
//        Serial1.println(adsVDD.readADC_SingleEnded(1), DEC);
// //        Serial1.print(", sg_2=");
// //        Serial1.print(adsVDD.readADC_SingleEnded(2), DEC);
// //        Serial1.print(", sg_3=");
// //        Serial1.println(adsVDD.readADC_SingleEnded(3), DEC);
//    }
//}

void setup() {
    // enable LED indication
    pinMode(LED, OUTPUT);

    // enable pins for PWM
    pinMode(PWM_PIN0, OUTPUT);
    pinMode(PWM_PIN1, OUTPUT);
    pinMode(PWM_PIN2, OUTPUT);
    pinMode(PWM_PIN3, OUTPUT);
    // enable PWM for fan1 and fan2 pins
    pinMode(PIN_FAN1, OUTPUT);
    pinMode(PIN_FAN2, OUTPUT);

    // enable stepper pins
    pinMode(STEPPER1_DIR_PIN, OUTPUT);
    pinMode(STEPPER1_STEP_PIN, OUTPUT);
    pinMode(STEPPER2_DIR_PIN, OUTPUT);
    pinMode(STEPPER2_STEP_PIN, OUTPUT);

    // Configure the first stepper
    stepper0.setEnablePin(STEPPERS_ENABLE);
    stepper0.setPinsInverted(false, false, true); //dir stp enable
    stepper0.setMinPulseWidth(PULSE_WIDTH);
    stepper0.setMaxSpeed(MAX_SPEED);
    stepper0.setAcceleration(ACCELERATION);

    // Configure the second stepper
    stepper1.setEnablePin(STEPPERS_ENABLE);
    stepper1.setPinsInverted(false, false, true); //dir stp enable //does not work, enabling manually
    stepper1.setMinPulseWidth(PULSE_WIDTH);
    stepper1.setMaxSpeed(MAX_SPEED);
    stepper1.setAcceleration(ACCELERATION);

    // Then give them to MultiStepper to manage
    steppers.addStepper(stepper0);
    steppers.addStepper(stepper1);


    stepperPositions[0] = 0;
    stepperPositions[1] = 0;

    // start CAN port for the CAN1 connection on the 22/23 ports
    can1.begin();
    can1.setBaudRate(500000);
    can1.enableMBInterrupts();
    can1.onReceive(canMessagesSniff);
    can1.mailboxStatus();
    isCANMessageAvailable = false;

    // start both ADS1115 analog-digital sensor
    adsGND.begin();
    adsVDD.begin();

    // set initial frequency 1kHz for PWM pins
    analogWriteFrequency(PWM_PIN0, 1000);
    analogWriteFrequency(PWM_PIN1, 1000);
    analogWriteFrequency(PWM_PIN2, 1000);
    analogWriteFrequency(PWM_PIN3, 1000);
    // also set initial frequency 1kHz for the FAN1, and FAN2 pins.
    analogWriteFrequency(PIN_FAN1, 1000);
    analogWriteFrequency(PIN_FAN2, 1000);

    // set initial resolution for PWM pins
    analogWriteResolution(15);

    // set initial value for speed.
    analogWrite(PIN_FAN1, 0);
    analogWrite(PIN_FAN2, 0);

    // add stop contacts
    // Attach the debouncer to a pin with INPUT_PULLUP mode
    debouncerTop.attach(STOP_BOT_PIN, INPUT_PULLUP);
    debouncerTop.interval(STOP_BOT_DEBOUNCE);
    attachInterrupt(STOP_BOT_PIN, stopBottomContactISR, CHANGE);
    // Attach the debouncer to a pin with INPUT_PULLUP mode
    debouncerBottom.attach(STOP_TOP_PIN, INPUT_PULLUP);
    debouncerBottom.interval(STOP_TOP_DEBOUNCE);
    attachInterrupt(STOP_TOP_PIN, stopTopContactISR, CHANGE);

//    // Attach the interrupt to a pin with ADC_CS1 status and set level for interrupting
//    pinMode(ADC_CS1_ALERT_PIN, INPUT_PULLUP);
//    adsGND.startComparator_SingleEnded(1, ADC_CS1_THRESHOLD_LEVEL);
//    attachInterrupt(ADC_CS1_ALERT_PIN, onLeverExceedingADC_CS1, CHANGE);
//    // Attach the interrupt to a pin with ADC_CS2 status and set level for interrupting
//    pinMode(ADC_CS2_ALERT_PIN, INPUT_PULLUP);
//    adsVDD.startComparator_SingleEnded(1, ADC_CS2_THRESHOLD_LEVEL);
//    attachInterrupt(ADC_CS2_ALERT_PIN, onLeverExceedingADC_CS2, CHANGE);

    // start serial1 ports
    Serial1.begin(115200);

    // wait for serial1 port to connect. Needed for native USB port only
    while (!Serial1) { ;
    }

    // a bit debug
    Serial1.println("Serial1 enabled!!!!");

    // start independent thread for the CAN1 processing
    std::thread th1(messageParseCANByThread);
    th1.detach();

    //initialize the PID variables and turn on the heater0 PID
    inputPIDHeater0 = 0;
    setpointPIDHeater0 = 0;
    heaterPID0.SetMode(AUTOMATIC);
    std::thread th2(heater0PIDControllerThread);
    th2.detach();

//    std::thread th3(leverPrinterThread);
//    th3.detach();
}

void loop() {
    can1.events();

    //Blink LED
    int time = round(millis() / 2000.0);
    if (time % 2 == 0) {
        digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    } else {
        digitalWrite(LED, LOW);   // turn the LED on (HIGH is the voltage level)
    }
}