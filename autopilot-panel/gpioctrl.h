#ifndef _GPIOCTRL_H_
#define _GPIOCTRL_H_

#include <climits>
#include <thread>
#include "globals.h"

extern globalVars globals;

// Set maximum number of controls
const int MaxControls = 20;

enum pinType {
    Rot1 = 0,
    Rot2 = 1,
    Push = 2,
    Toggle = 3,
    Led = 4
};

class gpioctrl
{
private:
    std::thread *watcherThread = NULL;

public:
    int controlCount = 0;
    int gpio[MaxControls][5];   // One slot for each pinType
    int rotateValue[MaxControls];
    int pushValue[MaxControls];
    int toggleValue[MaxControls];
    int lastRotateValue[MaxControls];
    int lastPushValue[MaxControls];
    int lastRotateState[MaxControls];
    int lastPushState[MaxControls];
    bool clockwise[MaxControls];

public:
    gpioctrl(bool initWiringPi);
    ~gpioctrl();
    int getSetting(const char* control, const char* controlType, const char* attribute);
    int addControl();
    int addRotaryEncoder(const char* controlName);
    int addButton(const char* controlName);
    int addSwitch(const char* controlName);
    int addLamp(const char* controlName);
    int readRotation(int control);
    int readPush(int control);
    void writeLed(int control, bool on);

private:
    void validateControl(const char* controlName, int control);
    void initPin(int pin, bool isInput);
};

#endif // _GPIOCTRL_H_
