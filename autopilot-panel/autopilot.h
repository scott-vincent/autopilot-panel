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
    bool airliner = false;
    sevensegment* sevenSegment;

    unsigned char display1[8];
    unsigned char display2[8];
    unsigned char display3[8];
    AutopilotSpd autopilotSpd;
    AutopilotHdg autopilotHdg;
    AutopilotAlt autopilotAlt = AltHold;
    bool showMach = false;
    double mach;
    double speed;
    int heading;
    int altitude;
    int verticalSpeed;
    int fpaX10 = 0;
    bool apEnabled;
    bool fdEnabled;
    bool athrEnabled;
    bool locEnabled;
    bool apprEnabled;
    double lastSetHeading = -1;
    double setVerticalSpeed = 0;
    double setAltitude;
    bool managedSpeed = false;
    bool managedHeading = false;
    bool managedAltitude = false;
    int orbit = 0;  // 1 = left orbit, 2 = right orbit
    int orbitDelay = 0;

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
    int prevSpdValSb = 0;
    int prevSpdPushSb = 0;
    int prevHdgValSb = 0;
    int prevHdgPushSb = 0;
    int prevAltValSb = 0;
    int prevAltPushSb = 0;
    int prevVsValSb = 0;
    int prevVsPushSb = 0;
    int prevApPushSb = 0;
    int prevLocPushSb = 0;
    int prevApprPushSb = 0;

    time_t lastSpdAdjust = 0;
    time_t lastSpdPush = 0;
    time_t lastHdgAdjust = 0;
    time_t lastHdgPush = 0;
    time_t lastAltAdjust = 0;
    time_t lastAltPush = 0;
    time_t lastVsAdjust = 0;
    time_t lastVsPush = 0;
    time_t lastApAdjust = 0;
    time_t lastFdAdjust = 0;
    time_t lastAthrAdjust = 0;
    time_t lastLocAdjust = 0;
    time_t lastApprAdjust = 0;
    time_t now;

public:
    autopilot();
    void render();
    void update();

private:
    void sendEvent(EVENT_ID id, double value);
    void addGpio();
    void gpioSpeedInput();
    void gpioHeadingInput();
    void gpioAltitudeInput();
    void gpioVerticalSpeedInput();
    void gpioButtonsInput();
    void machSwap();
    void toggleFlightDirector();
    void captureInitial();
    void manSelSpeed();
    void manSelHeading();
    void manSelAltitude();
    void selectedVs();
    void captureCurrent();
    void captureVerticalSpeed();
    void restoreVerticalSpeed();
    int adjustSpeed(int adjust);
    double adjustMach(int adjust);
    int adjustHeading(int adjust);
    int adjustAltitude(int adjust);
    int adjustVerticalSpeed(int adjust);
    double adjustFpa(int adjust);
    void continueOrbit();
};

#endif // _AUTOPILOT_H
