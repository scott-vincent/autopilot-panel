#ifndef _AUTPILOT_H_
#define _AUTPILOT_H_

#include "simvars.h"
#include "sevensegment.h"

class autopilot
{
private:
    enum AutopilotSpd {
        NoSpd,
        SpdHold
    };

    enum AutopilotHdg {
        NoHdg,
        HdgSet,
        LevelFlight
    };

    enum AutopilotAlt {
        NoAlt,
        AltHold,
        PitchHold,
        VerticalSpeedHold
    };

    SimVars* simVars;
    Aircraft loadedAircraft = UNDEFINED;
    sevensegment* sevenSegment;

    AutopilotSpd autopilotSpd;
    AutopilotHdg autopilotHdg;
    AutopilotAlt autopilotAlt = AltHold;
    bool showMach = false;
    bool showSpeed = false;
    bool showHeading = false;
    bool showAltitude = true;
    bool showVerticalSpeed = false;
    int machX100;
    int speed;
    int heading;
    int altitude;
    int verticalSpeed;
    int prevSpeed;
    int prevHeading;
    int prevAltitude;
    int prevVerticalSpeed;
    double setVerticalSpeed = 0;
    double setAltitude;
    bool managedSpeed = true;
    bool managedHeading = true;
    bool managedAltitude = true;

    // Hardware controls
    int speedControl = -1;
    int headingControl = -1;
    int altitudeControl = -1;
    int verticalSpeedControl = -1;
    int autopilotControl = -1;
    int flightDirectorControl = -1;
    int machControl = -1;
    int autothrottleControl = -1;
    int localiserControl = -1;
    int approachControl = -1;

    int prevSpdVal = 0;
    int prevSpdPush = 0;
    int spdSetSel = 0;
    int prevHdgVal = 0;
    int prevHdgPush = 0;
    int hdgSetSel = 0;
    int prevAltVal = 0;
    int prevAltPush = 0;
    int altSetSel = 0;
    int prevVsVal = 0;
    int prevVsPush = 0;
    int prevApPush = 0;
    int prevFdPush = 0;
    int prevMachPush = 0;
    int prevAthrPush = 0;
    int prevLocPush = 0;
    int prevApprPush = 0;

    time_t lastSpdAdjust = 0;
    time_t lastSpdPush = 0;
    time_t lastHdgAdjust = 0;
    time_t lastHdgPush = 0;
    time_t lastAltAdjust = 0;
    time_t lastAltPush = 0;
    time_t lastVsAdjust = 0;
    time_t now;

public:
    autopilot();
    void render();
    void update();

private:
    void addGpio();
    void gpioSpeedInput();
    void gpioHeadingInput();
    void gpioAltitudeInput();
    void gpioVerticalSpeedInput();
    void gpioButtonsInput();
    void machSwap();
    void toggleFlightDirector();
    void manSelSpeed();
    void manSelHeading();
    void manSelAltitude();
    void captureCurrent();
    void captureVerticalSpeed();
    void restoreVerticalSpeed();
    int adjustSpeed(int adjust);
    double adjustMach(int adjust);
    int adjustHeading(int adjust);
    int adjustAltitude(int adjust);
    int adjustVerticalSpeed(int adjust);
};

#endif // _AUTOPILOT_H
