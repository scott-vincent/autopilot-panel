#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <wiringPi.h>
#include "settings.h"
#include "gpioctrl.h"

const char* GpioGroup = "GPIO";
const char* RotaryEncoderGroup = "RotaryEncoder";
const char* SwitchGroup = "Switch";

void watcher(gpioctrl*);

gpioctrl::gpioctrl()
{
    // Use BCM GPIO pin numbers
    wiringPiSetupGpio();
}

gpioctrl::~gpioctrl()
{
    if (watcherThread) {
        // Wait for thread to exit
        watcherThread->join();
    }
}

int gpioctrl::getSetting(const char* controlName, const char *controlType, const char *attribute)
{
    char settingGroup[256];

    sprintf(settingGroup, "%s/%s/%s", GpioGroup, controlName, controlType);
    return globals.allSettings->getInt(settingGroup, attribute);
}

int gpioctrl::addRotaryEncoder(const char *controlName)
{
    if (controlCount >= MaxControls) {
        printf("Maximum number of GPIO controls exceeded\n");
        exit(1);
    }

    gpio[controlCount][Rot1] = getSetting(controlName, RotaryEncoderGroup, "Rot1");
    gpio[controlCount][Rot2] = getSetting(controlName, RotaryEncoderGroup, "Rot2");
    gpio[controlCount][Push] = getSetting(controlName, RotaryEncoderGroup, "Push");
    gpio[controlCount][Led] = INT_MIN;

    char msg[256];
    if (gpio[controlCount][Rot1] != INT_MIN && gpio[controlCount][Rot2] != INT_MIN) {
        initPin(gpio[controlCount][Rot1], true);
        initPin(gpio[controlCount][Rot2], true);
        sprintf(msg, "Add %s rotary encoder: GPIO%d, GPIO%d",
            controlName, gpio[controlCount][Rot1], gpio[controlCount][Rot2]);
    }
    else if (gpio[controlCount][Rot1] != INT_MIN || gpio[controlCount][Rot2] != INT_MIN) {
        printf("Must specify both Rot1 and Rot2 (or neither) for control: %s\n", controlName);
        exit(1);
    }
    else {
        msg[0] = '\0';
    }

    if (gpio[controlCount][Push] != INT_MIN) {
        initPin(gpio[controlCount][Push], true);
        if (msg[0] == '\0') {
            sprintf(msg, "Add %s rotary encoder push: GPIO%d", 
                controlName, gpio[controlCount][Push]);
        }
        else {
            char addMsg[256];
            sprintf(addMsg, " with push: GPIO%d", gpio[controlCount][Push]);
            strcat(msg, addMsg);
        }
    }

    if (msg[0] != '\0') {
        printf("%s\n", msg);
    }

    rotateValue[controlCount] = 0;
    pushValue[controlCount] = 0;
    lastRotateValue[controlCount] = -1;
    lastPushValue[controlCount] = -1;
    lastRotateState[controlCount] = -1;
    lastPushState[controlCount] = -1;
    clockwise[controlCount] = true;

    controlCount++;
    return controlCount - 1;
}

int gpioctrl::addSwitch(const char* controlName)
{
    if (controlCount >= MaxControls) {
        printf("Maximum number of GPIO controls exceeded\n");
        exit(1);
    }

    gpio[controlCount][Rot1] = INT_MIN;
    gpio[controlCount][Rot2] = INT_MIN;
    gpio[controlCount][Push] = getSetting(controlName, SwitchGroup, "Push");
    gpio[controlCount][Led] = getSetting(controlName, SwitchGroup, "Led");

    char msg[256];
    if (gpio[controlCount][Push] != INT_MIN) {
        initPin(gpio[controlCount][Push], true);
        sprintf(msg, "Add %s switch: GPIO%d", controlName, gpio[controlCount][Push]);
    }
    else {
        msg[0] = '\0';
    }

    if (gpio[controlCount][Led] != INT_MIN) {
        initPin(gpio[controlCount][Led], false);
        if (msg[0] == '\0') {
            sprintf(msg, "Add %s led: GPIO%d", controlName, gpio[controlCount][Led]);
        }
        else {
            char addMsg[256];
            sprintf(addMsg, " with led: GPIO%d", gpio[controlCount][Led]);
            strcat(msg, addMsg);
        }
    }

    if (msg[0] != '\0') {
        printf("%s\n", msg);
    }

    pushValue[controlCount] = 0;
    lastPushValue[controlCount] = -1;
    lastPushState[controlCount] = -1;

    controlCount++;
    return controlCount - 1;
}

void gpioctrl::initPin(int pin, bool isInput)
{
    if (!isInput) {
        pinMode(pin, OUTPUT);
        return;
    }

    pinMode(pin, INPUT);

    char command[256];

    // NOTE: pullUpDnControl does not work on RasPi4 so have
    // to use raspi-gpio command line to pull up resistors.
    sprintf(command, "raspi-gpio set %d pu", pin);

    if (system(command) != 0) {
        printf("Failed to run raspi-gpio command\n");
        exit(1);
    }
}

int gpioctrl::readRotation(int control)
{
    // Disabled if no GPIO specified in settings file
    if (gpio[control][Rot1] == INT_MIN) {
        return INT_MIN;
    }

    int newVal = INT_MIN;

    if (watcherThread) {
        if (control >= 0 && rotateValue[control] != lastRotateValue[control]) {
            newVal = rotateValue[control];
            lastRotateValue[control] = newVal;
        }
    }
    else {
        // Start monitoring controls on first read
        watcherThread = new std::thread(watcher, this);
    }

    return newVal;
}

int gpioctrl::readPush(int control)
{
    // Disabled if no GPIO specified in settings file
    if (gpio[control][Push] == INT_MIN) {
        return INT_MIN;
    }

    int newVal = INT_MIN;

    if (watcherThread) {
        if (control >= 0 && pushValue[control] != lastPushValue[control]) {
            newVal = pushValue[control];
            lastPushValue[control] = newVal;
        }
    }
    else {
        // Start monitoring controls on first read
        watcherThread = new std::thread(watcher, this);
    }

    return newVal;
}

void gpioctrl::writeLed(int control, bool on)
{
    // Disabled if no GPIO specified in settings file
    if (gpio[control][Led] == INT_MIN) {
        return;
    }

    if (on) {
        digitalWrite(gpio[control][Led], 1);
    }
    else {
        digitalWrite(gpio[control][Led], 0);
    }
}

/// <summary>
/// Need to monitor hardware controls on a separate thread
/// at constant small intervals so we don't miss any events.
/// Need accurate readings to determine which way a rotary
/// encoder is being rotated.
/// </summary>
void watcher(gpioctrl *t)
{
    int state;

    while (!globals.quit) {
        for (int control = 0; control < t->controlCount; control++) {
            // Check control rotation
            if (t->gpio[control][Rot1] != INT_MIN) {
                state = digitalRead(t->gpio[control][Rot1]) + digitalRead(t->gpio[control][Rot2]) * 2;
                if (state != t->lastRotateState[control]) {
                    if ((t->lastRotateState[control] == 0 && state == 2) ||
                        (t->lastRotateState[control] == 2 && state == 3) ||
                        (t->lastRotateState[control] == 3 && state == 1) ||
                        (t->lastRotateState[control] == 1 && state == 0))
                    {
                        // Rotating clockwise
                        t->clockwise[control] = true;
                        t->rotateValue[control]++;
                    }
                    else if ((t->lastRotateState[control] == 0 && state == 1) ||
                        (t->lastRotateState[control] == 1 && state == 3) ||
                        (t->lastRotateState[control] == 3 && state == 2) ||
                        (t->lastRotateState[control] == 2 && state == 0))
                    {
                        // Rotating anti-clockwise
                        t->clockwise[control] = false;
                        t->rotateValue[control]--;
                    }
                    else if (t->lastRotateState[control] != -1) {
                        // Missed rotation so assume same direction as previous
                        if (t->clockwise[control]) {
                            t->rotateValue[control]++;
                        }
                        else {
                            t->rotateValue[control]--;
                        }
                    }

                    t->lastRotateState[control] = state;
                }
            }

            // Check control push
            if (t->gpio[control][Push] != INT_MIN) {
                state = digitalRead(t->gpio[control][Push]);
                if (state != t->lastPushState[control]) {
                    // If pressed increment value to next even number
                    // otherwise increment value to next odd number.
                    // This ensures no presses can be 'lost'.
                    if (state == 0) {
                        if (t->pushValue[control] % 2 == 1) t->pushValue[control]++; else t->pushValue[control] += 2;
                    }
                    else {
                        if (t->pushValue[control] % 2 == 0) t->pushValue[control]++; else t->pushValue[control] += 2;
                    }
                }
            }
        }

        delay(1);
    }
}
