#include <Arduino.h>
#include <LowPower.h>
#include <IRremote.hpp>
#include <Servo.h>

#define DEBUG 0

#define PIN_FAN_LEFT 2  // digital
#define PIN_FAN_RIGHT 3 // digital

#define PIN_MOTOR_LEFT 6  // digital
#define PIN_MOTOR_RIGHT 7 // digital

#define PIN_ULTR_TRIG 12 // digital
#define PIN_ULTR_ECHO 13 // digital

#define PIN_IR 11    // digital
#define PIN_SERVO 10 // analog

#define CMD_STOP 0x1c    // 5
#define CMD_FORWARD 0x18 // 2
#define CMD_LEFT 0x08    // 4
#define CMD_RIGHT 0x5A   // 6

#define CMD_OPEN 0x09      // arrow up
#define CMD_CLOSE 0x07     // arrow down
#define CMD_CLAW_STOP 0x15 // vol-

#define CMD_AUTO 0x45   // POWER
#define CMD_MANUAL 0x46 // VOL+

enum class Direction
{
    STOP,
    LEFT,
    FORWARD,
    RIGHT
};

enum class DetectionState
{
    INIT,
    SEARCH,
    HUNT
};

enum class ClawState
{
    PULL,
    STOP,
    RELEASE,
};

DetectionState state = DetectionState::INIT;
static Servo eyeMotor;
static bool angleUp = true;
static bool autoMode = true;
static bool lockedIn = false;
static struct
{
    float distance;
    int8_t angle;
} target;

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;

    pinMode(PIN_FAN_LEFT, OUTPUT);
    pinMode(PIN_FAN_RIGHT, OUTPUT);

    pinMode(PIN_MOTOR_LEFT, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT, OUTPUT);

    pinMode(PIN_ULTR_TRIG, OUTPUT);
    pinMode(PIN_ULTR_ECHO, INPUT);

    eyeMotor.attach(PIN_SERVO);

    IrReceiver.begin(PIN_IR);
    IrReceiver.start();
}

void rotateClaw(ClawState state)
{
    switch (state)
    {
    case ClawState::PULL:
        digitalWrite(PIN_MOTOR_LEFT, HIGH);
        digitalWrite(PIN_MOTOR_RIGHT, LOW);
        break;
    case ClawState::STOP:
        digitalWrite(PIN_MOTOR_LEFT, LOW);
        digitalWrite(PIN_MOTOR_RIGHT, LOW);
        break;
    case ClawState::RELEASE:
        digitalWrite(PIN_MOTOR_LEFT, LOW);
        digitalWrite(PIN_MOTOR_RIGHT, HIGH);
        break;
    }
}

float readDistanceOnce()
{
    digitalWrite(PIN_ULTR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTR_TRIG, LOW);

    float duration = pulseIn(PIN_ULTR_ECHO, HIGH);

    return (duration * 0.0343) / 2.;
}

float readDistance(uint8_t tries = 4)
{
    float sum = 0.0f;

    for (uint8_t i = 0; i < tries; ++i)
    {
        sum += readDistanceOnce();
    }

    return sum / tries;
}

void sleep(period_t dur)
{
    if (!DEBUG)
    {
        LowPower.powerDown(dur, ADC_ON, BOD_ON);
        return;
    }

    switch (dur)
    {
    case SLEEP_15MS:
        delay(15);
        break;
    case SLEEP_30MS:
        delay(30);
        break;
    case SLEEP_60MS:
        delay(60);
        break;
    case SLEEP_120MS:
        delay(120);
        break;
    case SLEEP_250MS:
        delay(250);
        break;
    case SLEEP_500MS:
        delay(500);
        break;
    case SLEEP_1S:
        delay(1000);
        break;
    case SLEEP_2S:
        delay(2000);
        break;
    case SLEEP_4S:
        delay(4000);
        break;
    case SLEEP_8S:
        delay(8000);
        break;
    case SLEEP_FOREVER:
        delay(9999999999);
        break;
    }
}

void move(Direction dir)
{
    digitalWrite(PIN_FAN_LEFT, (dir == Direction::FORWARD || dir == Direction::RIGHT) ? HIGH : LOW);
    digitalWrite(PIN_FAN_RIGHT, (dir == Direction::FORWARD || dir == Direction::LEFT) ? HIGH : LOW);
}

void handleRemote(IRData data)
{
    if (data.protocol != NEC)
        return;

    switch (data.command)
    {
    case CMD_AUTO:
        autoMode = true;
        state = DetectionState::INIT;
        break;
    case CMD_MANUAL:
        autoMode = false;
        break;
    }

    if (autoMode)
        return;

    switch (data.command)
    {
    case CMD_STOP:
        move(Direction::STOP);
        break;
    case CMD_FORWARD:
        move(Direction::FORWARD);
        break;
    case CMD_LEFT:
        move(Direction::LEFT);
        break;
    case CMD_RIGHT:
        move(Direction::RIGHT);
        break;
    case CMD_OPEN:
        rotateClaw(ClawState::PULL);
        sleep(SLEEP_500MS);
        rotateClaw(ClawState::STOP);
        break;
    case CMD_CLOSE:
        rotateClaw(ClawState::RELEASE);
        sleep(SLEEP_500MS);
        rotateClaw(ClawState::STOP);
        break;
    case CMD_CLAW_STOP:
        rotateClaw(ClawState::STOP);
        break;
    default:
        break;
    }
}

void stepRotation(int8_t curAngle)
{
    if (!angleUp && curAngle <= 0)
        angleUp = true;
    if (angleUp && curAngle >= 120)
        angleUp = false;

    if (angleUp)
        eyeMotor.write(curAngle + 10);
    else
        eyeMotor.write(curAngle - 10);
}

void stateInit()
{
    eyeMotor.write(0);
    angleUp = true;
    state = DetectionState::SEARCH;
    target.angle = 0;
    target.distance = 9999999.0f;
    lockedIn = false;
    move(Direction::STOP);
}

void stateSearch()
{
    int8_t angle = eyeMotor.read();

    // done searching
    if (angle == 0 && angleUp == false)
    {
        if (target.distance > 300)
            state = DetectionState::INIT;
        else
            state = DetectionState::HUNT;
        return;
    }

    float distance = readDistance();
    if (20 < distance && distance < target.distance)
    {
        target.distance = distance;
        target.angle = angle;
    }

    stepRotation(angle);
}

void stateHunt()
{
    eyeMotor.write(60);

    float diff = readDistance() - target.distance;
    diff = abs(diff);

    if (!lockedIn)
    {
        if (diff < 10.0f)
            lockedIn = true;
        else
            move(target.angle < 60 ? Direction::RIGHT : Direction::LEFT);
    }
    else
    {
        if (diff < 0.5)
        {
            state = DetectionState::INIT;
        }
        else
        {
            move(Direction::FORWARD);
        }
    }
}

void loop()
{
    if (IrReceiver.decode())
    {
        IRData data = IrReceiver.decodedIRData;
        Serial.print("Got cmd (hex): ");
        Serial.println(IrReceiver.decodedIRData.command, HEX);
        IrReceiver.resume();

        handleRemote(data);
    }

    if (autoMode)
    {
        switch (state)
        {
        case DetectionState::INIT:
            stateInit();
            break;
        case DetectionState::SEARCH:
            stateSearch();
            break;
        case DetectionState::HUNT:
            stateHunt();
            break;
        }
    }

    sleep(SLEEP_15MS);
}