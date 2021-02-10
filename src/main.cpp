// TODO! make nice code view
// TODO! add a few code comments

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Adafruit_ADS1015.h>
#include <Bounce2.h>

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

constexpr uint32_t DEVICE_CAN_ID = 0x640 + BOARD_ID;
constexpr uint32_t START_MOVING_ID = 0x080;
constexpr uint32_t ADDRESS_RECEIVE_OK_ID = 0x5C0 + BOARD_ID;
constexpr uint32_t VELOCITY_POSITION_STEPPER_0_DEV_ID = 0x440;
constexpr uint32_t VELOCITY_POSITION_STEPPER_1_DEV_ID = 0x441;
constexpr uint32_t VELOCITY_POSITION_OK_ID = 0x3C0;
constexpr uint32_t HOMING_RESULT_CAN_ID = 0x3C0;

constexpr uint32_t MESSAGE_TYPE_STEPPER_MOVING = 0x23;
constexpr uint32_t SUB_REGISTER_ADDRESS = 0x00;

constexpr uint32_t CODE_SET_SPEED = 0x8160;
constexpr uint32_t CODE_SET_ACCELERATION = 0x8360;
constexpr uint32_t CODE_SET_POSITION = 0x7A60;
constexpr uint32_t CODE_SET_HOME_OFFSET = 0x7C60;
constexpr uint32_t CODE_SET_HOME_SPEED = 0x9960;
constexpr uint32_t CODE_SET_GO_HOME = 0x6060;

constexpr uint32_t MESSAGE_TYPE_REQUEST_CHANNEL_AD_DATA = 0x40;
constexpr uint32_t MESSAGE_TYPE_RETURN_CHANNEL_AD_DATA = 0x43;
constexpr uint32_t CODE_CHANNEL_AD = 0x0061;

constexpr uint32_t MESSAGE_TYPE_SET_PWM = 0x23;
constexpr uint32_t CODE_SET_PWM = 0x0062;

constexpr uint32_t RESOLUTION_PWM_BYTE = 2;
constexpr float RESOLUTION_PWM_BIT = 32767.0;

constexpr uint32_t CODE_GET_ENDSTOP_STATES = 0xFD60;
constexpr uint32_t MESSAGE_TYPE_REQUEST =    0x40;
constexpr uint32_t MESSAGE_TYPE_REPORT =     0x43;


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;
AccelStepper stepper0(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
MultiStepper steppers;
long stepperPositions[2];
Adafruit_ADS1115 adsGND(ADS1115_I2C_ADDRESS_GND);
Adafruit_ADS1115 adsVDD(ADS1115_I2C_ADDRESS_VDD);

float justMovingSpeedStepper1 = 0.0;
float justMovingSpeedStepper2 = 0.0;

long homePositionOffset = 0;
float homeMovingSpeed = 100.0;

bool isTopNotStopped;

/// this is a debouncer, which prevent mechanical noise on stop-end activation
Bounce debouncerTop = Bounce();
Bounce debouncerBottom = Bounce();

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

void setup() {
    // enable LED indication
    pinMode(LED, OUTPUT);

    // enable pins for PWM
    pinMode(PWM_PIN0, OUTPUT);
    pinMode(PWM_PIN1, OUTPUT);
    pinMode(PWM_PIN2, OUTPUT);
    pinMode(PWM_PIN3, OUTPUT);

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

    // start both ADS1115 analog-digital sensor
    adsGND.begin();
    adsVDD.begin();

    // set initial frequency 1kHz for PWM pins
    analogWriteFrequency(PWM_PIN0, 1000);
    analogWriteFrequency(PWM_PIN1, 1000);
    analogWriteFrequency(PWM_PIN2, 1000);
    analogWriteFrequency(PWM_PIN3, 1000);

    // set initial resolution for PWM pins
    analogWriteResolution(15);

    // add stop contacts
    // Attach the debouncer to a pin with INPUT_PULLUP mode
    debouncerTop.attach(STOP_BOT_PIN, INPUT_PULLUP);
    debouncerTop.interval(STOP_BOT_DEBOUNCE);
    attachInterrupt(STOP_BOT_PIN, stopBottomContactISR, CHANGE);
    // Attach the debouncer to a pin with INPUT_PULLUP mode
    debouncerBottom.attach(STOP_TOP_PIN, INPUT_PULLUP);
    debouncerBottom.interval(STOP_TOP_DEBOUNCE);
    attachInterrupt(STOP_TOP_PIN, stopTopContactISR, CHANGE);

    // start serial1 ports
    Serial1.begin(115200);

    // wait for serial1 port to connect. Needed for native USB port only
    while (!Serial1) { ;
    }

    // a bit debug
    Serial1.println("Serial1 enabled!!!!");
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
        if ((VELOCITY_POSITION_STEPPER_0_DEV_ID == msg.id)
            || (VELOCITY_POSITION_STEPPER_1_DEV_ID == msg.id)) {
            // get message data (position) from code
            uint32_t messageDataPosition = msg.buf[3] << 24;
            messageDataPosition |= msg.buf[2] << 16;
            messageDataPosition |= msg.buf[1] << 8;
            messageDataPosition |= msg.buf[0];

            // get message data (speed=velocity) from code
            uint32_t messageDataVelocity = msg.buf[7] << 24;
            messageDataVelocity |= msg.buf[6] << 16;
            messageDataVelocity |= msg.buf[5] << 8;
            messageDataVelocity |= msg.buf[4];

            // set max speed to steppers and set position to steppers
            switch (msg.id) {
                case VELOCITY_POSITION_STEPPER_0_DEV_ID:
                    justMovingSpeedStepper1 = messageDataVelocity;
                    stepperPositions[0] = messageDataPosition;

                    // a bit debug
                    Serial1.print("MaxSpeed is set for stepper_0: ");
                    Serial1.println(justMovingSpeedStepper1);
                    Serial1.print("Position is set for stepper_0: ");
                    Serial1.println(stepperPositions[0]);
                    break;
                case VELOCITY_POSITION_STEPPER_1_DEV_ID:
                    justMovingSpeedStepper2 = messageDataVelocity;
                    stepperPositions[1] = messageDataPosition;

                    // a bit debug
                    Serial1.print("MaxSpeed is set for stepper_1: ");
                    Serial1.println(justMovingSpeedStepper2);
                    Serial1.print("Position is set for stepper_1: ");
                    Serial1.println(stepperPositions[1]);
                    break;
            }

            // send back received message, which confirm message getting.
            msg.id = VELOCITY_POSITION_OK_ID;
            can1.write(msg);
        } else if (DEVICE_CAN_ID == msg.id) {
            // send back received message, which confirm message getting.
            msg.id = ADDRESS_RECEIVE_OK_ID;
            // it doesn't send back the same message if it works with channels
            if (MESSAGE_TYPE_REQUEST_CHANNEL_AD_DATA != msg.buf[0]) {
                can1.write(msg);
            }

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

            int16_t adData = 0;

            float pwmValue;

            switch (messageCode) {
                case CODE_SET_SPEED:
                    justMovingSpeedStepper1 = messageData;
                    justMovingSpeedStepper2 = messageData;
                    Serial1.println("MaxSpeed is set");
                    break;
                case CODE_SET_ACCELERATION:
                    stepper0.setAcceleration(messageData);
                    stepper1.setAcceleration(messageData);
                    Serial1.println("Acceleration is set");
                    break;
                case CODE_SET_POSITION:
                    stepperPositions[0] = messageData;
                    stepperPositions[1] = messageData;
                    Serial1.println("Position is set");
                    break;
                case CODE_CHANNEL_AD:
                    // read data from interested sensor
                    switch (msg.buf[3]) {
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

                    msg.buf[0] = MESSAGE_TYPE_RETURN_CHANNEL_AD_DATA;
                    msg.buf[4] = adData;
                    msg.buf[5] = adData >> 8;
                    msg.buf[6] = 0x0;
                    msg.buf[7] = 0x0;
                    can1.write(msg);

                    // a bit debug messages to Serial1
                    Serial1.print("get AM");
                    Serial1.print(msg.buf[3], HEX);
                    Serial1.print(": ");
                    Serial1.println(adData);
                    break;
                case CODE_SET_PWM:
                    pwmValue = messageData;
                    pwmValue = (pwmValue * RESOLUTION_PWM_BIT) / 100;
                    switch (msg.buf[3]) {
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

                    // a bit debug messages to Serial1
                    Serial1.print("set PWM for PIN:");
                    Serial1.println();
                    break;
                case CODE_SET_HOME_OFFSET:
                    homePositionOffset = messageData;

                    // a bit debug messages to Serial1
                    Serial1.print("Offset near home position: ");
                    Serial1.println(homePositionOffset, DEC);
                    break;
                case CODE_SET_HOME_SPEED:
                    homeMovingSpeed = messageData;

                    // a bit debug messages to Serial1
                    Serial1.print("Moving speed to home: ");
                    Serial1.println(homeMovingSpeed, DEC);
                    break;
                case CODE_SET_GO_HOME:
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
                    while (isTopNotStopped && steppers.run()) {
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
                    while (isTopNotStopped && steppers.run()) {
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
                    while (steppers.run()) { ; }
                    stepper0.disableOutputs();
                    stepper1.disableOutputs();

                    // return a new current position for the master device
                    msg.id = HOMING_RESULT_CAN_ID;
                    // set position
                    msg.buf[0] = stepper0.currentPosition();
                    msg.buf[1] = stepper0.currentPosition() >> 8;
                    msg.buf[2] = stepper0.currentPosition() >> 16;
                    msg.buf[3] = stepper0.currentPosition() >> 24;
                    // set speed
                    msg.buf[4] = (int32_t) homeMovingSpeed;
                    msg.buf[5] = ((int32_t) homeMovingSpeed) >> 8;
                    msg.buf[6] = ((int32_t) homeMovingSpeed) >> 16;
                    msg.buf[7] = ((int32_t) homeMovingSpeed) >> 24;
                    // send a message to the CAN1
                    can1.write(msg);

                    // a bit debug
                    Serial1.println("Homing is DONE!");
                    break;
                case CODE_GET_ENDSTOP_STATES:
                    // prepare a report message and read current PINs states
                    msg.id = ADDRESS_RECEIVE_OK_ID;
                    msg.buf[0] = MESSAGE_TYPE_REPORT;
                    msg.buf[3] = 0x00;
                    msg.buf[4] = 0x00;
                    msg.buf[5] = 0x00;
                    msg.buf[6] = (digitalRead(STOP_BOT_PIN) << 1) + digitalRead(STOP_TOP_PIN);
                    msg.buf[7] = 0x00;
                    can1.write(msg);

                    // a bit debug
                    Serial1.print("Send values for the pin_11: ");
                    Serial1.print(digitalRead(STOP_BOT_PIN), DEC);
                    Serial1.print(", for the pin_12: ");
                    Serial1.println(digitalRead(STOP_TOP_PIN));
                    break;
            }

        } else if (START_MOVING_ID == msg.id) {
            Serial1.println("Moving is START");
            stepper0.setMaxSpeed(justMovingSpeedStepper1);
            stepper1.setMaxSpeed(justMovingSpeedStepper2);
            stepper0.enableOutputs();
            stepper1.enableOutputs();
            steppers.moveTo(stepperPositions);
            bool isMoved = false;
            while (steppers.run()) {
                isMoved = true;
            }
            stepper0.disableOutputs();
            stepper1.disableOutputs();
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