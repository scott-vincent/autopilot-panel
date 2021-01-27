#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <set>
#include <wiringPi.h>
#include "settings.h"
#include "gpioctrl.h"

const char* GpioGroup = "GPIO";
const char* RotaryEncoderGroup = "RotaryEncoder";   // Rot1, Rot2, Push
const char* ButtonGroup = "Button";                 // Push, Led
const char* SwitchGroup = "Switch";                 // Toggle, Led
const char* LampGroup = "Lamp";                     // Led

// SPI GPIO pins
const int SPI_MOSI = 10;
const int SPI_SCLK = 11;
const int SPI_CE0 = 8;

void watcher(gpioctrl*);

gpioctrl::gpioctrl(bool initWiringPi)
{
    // Caller may want to initialise wiringPi themselves
    if (initWiringPi) {
        // Use BCM GPIO pin numbers
        wiringPiSetupGpio();
    }

    // Reserve pins for SPI channel 0 with no MISO
    printf("Added SPI CE0 with no MISO: GPIO%d, GPIO%d, GPIO%d\n", SPI_MOSI, SPI_SCLK, SPI_CE0);

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

int gpioctrl::addControl()
{
    if (controlCount >= MaxControls) {
        printf("Maximum number of GPIO controls exceeded\n");
        exit(1);
    }

    int num = controlCount;
    controlCount++;

    gpio[num][Rot1] = INT_MIN;
    gpio[num][Rot2] = INT_MIN;
    gpio[num][Push] = INT_MIN;
    gpio[num][Toggle] = INT_MIN;
    gpio[num][Led] = INT_MIN;

    rotateValue[num] = 0;
    pushValue[num] = 0;
    toggleValue[num] = 0;
    lastRotateValue[num] = -1;
    lastPushValue[num] = -1;
    lastRotateState[num] = -1;
    lastPushState[num] = -1;
    clockwise[num] = true;

    return num;
}

void gpioctrl::validateControl(const char* controlName, int control)
{
    std::set<int> usedPins;

    // Reserve SPI pins
    usedPins.insert(SPI_MOSI);
    usedPins.insert(SPI_SCLK);
    usedPins.insert(SPI_CE0);

    // Find all pins already in use
    for (int num = 0; num < controlCount; num++) {
        if (num != control) {
            if (gpio[num][Rot1] != INT_MIN) usedPins.insert(gpio[num][Rot1]);
            if (gpio[num][Rot2] != INT_MIN) usedPins.insert(gpio[num][Rot2]);
            if (gpio[num][Push] != INT_MIN) usedPins.insert(gpio[num][Push]);
            if (gpio[num][Toggle] != INT_MIN) usedPins.insert(gpio[num][Toggle]);
            if (gpio[num][Led] != INT_MIN) usedPins.insert(gpio[num][Led]);
        }
    }
    
    // Make sure at least one pin specified
    if (gpio[control][Rot1] == INT_MIN && gpio[control][Rot2] == INT_MIN
        && gpio[control][Push] == INT_MIN && gpio[control][Toggle] == INT_MIN
        && gpio[control][Led] == INT_MIN) {
        printf("Cannot add %s control as no settings specified\n", controlName);
        exit(1);
    }

    // Make sure new pins are unique
    if (usedPins.find(gpio[control][Rot1]) != usedPins.end()) {
        printf("Duplicate GPIO pin number specified for %s/Rot1\n", controlName);
        exit(1);
    }

    if (usedPins.find(gpio[control][Rot2]) != usedPins.end()) {
        printf("Duplicate GPIO pin number specified for %s/Rot2\n", controlName);
        exit(1);
    }

    if (usedPins.find(gpio[control][Push]) != usedPins.end()) {
        printf("Duplicate GPIO pin number specified for %s/Push\n", controlName);
        exit(1);
    }

    if (usedPins.find(gpio[control][Toggle]) != usedPins.end()) {
        printf("Duplicate GPIO pin number specified for %s/Toggle\n", controlName);
        exit(1);
    }

    if (usedPins.find(gpio[control][Led]) != usedPins.end()) {
        printf("Duplicate GPIO pin number specified for %s/Led\n", controlName);
        exit(1);
    }
}

int gpioctrl::addRotaryEncoder(const char *controlName)
{
    int newControl = addControl();

    gpio[newControl][Rot1] = getSetting(controlName, RotaryEncoderGroup, "Rot1");
    gpio[newControl][Rot2] = getSetting(controlName, RotaryEncoderGroup, "Rot2");
    gpio[newControl][Push] = getSetting(controlName, RotaryEncoderGroup, "Push");

    char msg[256];
    if (gpio[newControl][Rot1] != INT_MIN && gpio[newControl][Rot2] != INT_MIN) {
        initPin(gpio[newControl][Rot1], true);
        initPin(gpio[newControl][Rot2], true);
        sprintf(msg, "Added %s rotary encoder: GPIO%d, GPIO%d",
            controlName, gpio[newControl][Rot1], gpio[newControl][Rot2]);
    }
    else if (gpio[newControl][Rot1] != INT_MIN || gpio[newControl][Rot2] != INT_MIN) {
        printf("Must specify both Rot1 and Rot2 (or neither) for control: %s\n", controlName);
        exit(1);
    }
    else {
        msg[0] = '\0';
    }

    if (gpio[newControl][Push] != INT_MIN) {
        initPin(gpio[newControl][Push], true);
        if (msg[0] == '\0') {
            sprintf(msg, "Added %s rotary encoder push: GPIO%d", 
                controlName, gpio[newControl][Push]);
        }
        else {
            char addMsg[256];
            sprintf(addMsg, " with push: GPIO%d", gpio[newControl][Push]);
            strcat(msg, addMsg);
        }
    }

    if (msg[0] != '\0') {
        printf("%s\n", msg);
    }

    validateControl(controlName, newControl);
    return newControl;
}

int gpioctrl::addButton(const char* controlName)
{
    int newControl = addControl();

    gpio[newControl][Push] = getSetting(controlName, ButtonGroup, "Push");
    gpio[newControl][Led] = getSetting(controlName, ButtonGroup, "Led");

    char msg[256];
    if (gpio[newControl][Push] != INT_MIN) {
        initPin(gpio[newControl][Push], true);
        sprintf(msg, "Added %s button: GPIO%d", controlName, gpio[newControl][Push]);
    }
    else {
        msg[0] = '\0';
    }

    if (gpio[newControl][Led] != INT_MIN) {
        initPin(gpio[newControl][Led], false);
        if (msg[0] == '\0') {
            sprintf(msg, "Added %s led: GPIO%d", controlName, gpio[newControl][Led]);
        }
        else {
            char addMsg[256];
            sprintf(addMsg, " with led: GPIO%d", gpio[newControl][Led]);
            strcat(msg, addMsg);
        }
    }

    if (msg[0] != '\0') {
        printf("%s\n", msg);
    }

    validateControl(controlName, newControl);
    return newControl;
}

int gpioctrl::addSwitch(const char* controlName)
{
    int newControl = addControl();

    gpio[newControl][Toggle] = getSetting(controlName, SwitchGroup, "Toggle");
    gpio[newControl][Led] = getSetting(controlName, SwitchGroup, "Led");

    char msg[256];
    if (gpio[newControl][Toggle] != INT_MIN) {
        initPin(gpio[newControl][Toggle], true);
        sprintf(msg, "Added %s switch: GPIO%d", controlName, gpio[newControl][Toggle]);
    }
    else {
        msg[0] = '\0';
    }

    if (gpio[newControl][Led] != INT_MIN) {
        initPin(gpio[newControl][Led], false);
        if (msg[0] == '\0') {
            sprintf(msg, "Added %s led: GPIO%d", controlName, gpio[newControl][Led]);
        }
        else {
            char addMsg[256];
            sprintf(addMsg, " with led: GPIO%d", gpio[newControl][Led]);
            strcat(msg, addMsg);
        }
    }

    if (msg[0] != '\0') {
        printf("%s\n", msg);
    }

    validateControl(controlName, newControl);
    return newControl;
}

int gpioctrl::addLamp(const char* controlName)
{
    int newControl = addControl();

    gpio[newControl][Led] = getSetting(controlName, LampGroup, "Led");

    if (gpio[newControl][Led] != INT_MIN) {
        initPin(gpio[newControl][Led], false);
        printf("Added %s led: GPIO%d", controlName, gpio[newControl][Led]);
    }

    validateControl(controlName, newControl);
    return newControl;
}

void gpioctrl::initPin(int pin, bool isInput)
{
    char command[256];

    // NOTE: pullUpDnControl does not work on RasPi4 so have
    // to use raspi-gpio command line to pull up resistors.
    if (isInput) {
        sprintf(command, "raspi-gpio set %d pu", pin);
    }
    else {
        sprintf(command, "raspi-gpio set %d op", pin);
    }

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

                    t->lastPushState[control] = state;
                }
            }

            // Check control toggle
            if (t->gpio[control][Toggle] != INT_MIN) {
                t->toggleValue[control] = digitalRead(t->gpio[control][Toggle]);
            }
        }

        delay(1);
    }
}
