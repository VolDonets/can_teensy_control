// TODO! make nice code view
// TODO! add a few code comments

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Adafruit_ADS1015.h>
#include <Bounce2.h>

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

constexpr uint32_t DEVICE_CAN_ID = 0x640;
constexpr uint32_t START_MOVING_ID = 0x080;
constexpr uint32_t ADDRESS_RECEIVE_OK_ID = 0x5C0;
constexpr uint32_t VELOCITY_POSITION_DEV_ID = 0x440;
constexpr uint32_t VELOCITY_POSITION_OK_ID = 0x3C0;
constexpr uint32_t HOMING_RESULT_CAN_ID = 0x3C0;

constexpr uint32_t MESSAGE_TYPE_STEPPER_MOVING = 0x23;
constexpr uint32_t SUB_REGISTER_ADDRESS = 0x00;

constexpr uint32_t CODE_SET_SPEED =         0x8160;
constexpr uint32_t CODE_SET_ACCELERATION =  0x8360;
constexpr uint32_t CODE_SET_POSITION =      0x7A60;
constexpr uint32_t CODE_SET_HOME_TO_SPEED = 0x7C60;
constexpr uint32_t CODE_SET_HOME_OFFSET =   0x9960;
constexpr uint32_t CODE_SET_GO_HOME =       0x6060;

constexpr uint32_t MESSAGE_TYPE_REQUEST_CHANNEL_AD_DATA = 0x40;
constexpr uint32_t MESSAGE_TYPE_RETURN_CHANNEL_AD_DATA = 0x43;
constexpr uint32_t CODE_CHANNEL_AD = 0x0061;

constexpr uint32_t MESSAGE_TYPE_SET_PWM = 0x23;
constexpr uint32_t CODE_SET_PWM = 0x0062;

constexpr uint32_t RESOLUTION_PWM_BYTE = 2;
constexpr float RESOLUTION_PWM_BIT = 32767.0;


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
MultiStepper steppers;
long stepperPositions[2];
Adafruit_ADS1115 adsGND(ADS1115_I2C_ADDRESS_GND);
Adafruit_ADS1115 adsVDD(ADS1115_I2C_ADDRESS_VDD);

float justMovingSpeed = 0.0;

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
    // Attach the debouncer to a pin with INPUT_PULLDOWN mode
    debouncerTop.attach(STOP_BOT_PIN, INPUT_PULLDOWN);
    debouncerTop.interval(STOP_BOT_DEBOUNCE);
    attachInterrupt(STOP_BOT_PIN, stopBottomContactISR, CHANGE);
    // Attach the debouncer to a pin with INPUT_PULLDOWN mode
    debouncerBottom.attach(STOP_TOP_PIN, INPUT_PULLDOWN);
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
        if (VELOCITY_POSITION_DEV_ID == msg.id) {
            // send back received message, which confirm message getting.
            msg.id = VELOCITY_POSITION_OK_ID;
            can1.write(msg);

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

            // set max speed to steppers
            justMovingSpeed = messageDataVelocity;

            // set position to steppers
            stepperPositions[0] = messageDataPosition;
            stepperPositions[1] = messageDataPosition;

            // a bit debug
            Serial1.println("MaxSpeed is set");
            Serial1.println("Position is set");
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
                    justMovingSpeed = messageData;
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
                case CODE_SET_HOME_TO_SPEED:
                    homeMovingSpeed = messageData;

                    // a bit debug messages to Serial1
                    Serial1.print("Moving speed to home: ");
                    Serial1.println(homeMovingSpeed, DEC);
                    break;
                case CODE_SET_GO_HOME:
                    // set homeMovingSpeed as a maximum speed for the steppers
                    stepper1.setMaxSpeed(homeMovingSpeed);
                    stepper2.setMaxSpeed(homeMovingSpeed);

                    // move while no stop flag enabled, for this look function stopTopContactISR()
                    isTopNotStopped = true;
                    while (isTopNotStopped) {
                        stepperPositions[0] = stepper1.currentPosition() + 100;
                        stepperPositions[1] = stepper2.currentPosition() + 100;
                        steppers.moveTo(stepperPositions);
                        while (isTopNotStopped && steppers.run()){ ; }
                    }

                    // move while no stop flag disable, for this look function stopTopContactISR()
                    isTopNotStopped = true;
                    while (isTopNotStopped) {
                        stepperPositions[0] = stepper1.currentPosition() - 100;
                        stepperPositions[1] = stepper2.currentPosition() - 100;
                        steppers.moveTo(stepperPositions);
                        while (isTopNotStopped && steppers.run()) { ; }
                    }

                    // move from the stop point on the offset
                    stepperPositions[0] = stepper1.currentPosition() - homePositionOffset;
                    stepperPositions[1] = stepper2.currentPosition() - homePositionOffset;
                    steppers.moveTo(stepperPositions);
                    while (steppers.run()) { ; }

                    // return a new current position for the master device
                    msg.id = HOMING_RESULT_CAN_ID;
                    // set position
                    msg.buf[0] = stepper1.currentPosition();
                    msg.buf[1] = stepper1.currentPosition() >> 8;
                    msg.buf[2] = stepper1.currentPosition() >> 16;
                    msg.buf[3] = stepper1.currentPosition() >> 24;
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
            }

        } else if (START_MOVING_ID == msg.id) {
            Serial1.println("Moving is START");
            stepper1.setMaxSpeed(justMovingSpeed);
            stepper2.setMaxSpeed(justMovingSpeed);
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