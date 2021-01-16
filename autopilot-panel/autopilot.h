#ifndef _AUTPILOT_H_
#define _AUTPILOT_H_

#include "simvars.h"

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
    Aircraft loadedAircraft = NO_AIRCRAFT;

    AutopilotSpd autopilotSpd;
    AutopilotHdg autopilotHdg;
    AutopilotAlt autopilotAlt = AltHold;
    bool showMach = false;
    bool showHeading = false;
    bool showSpeed = false;
    bool showAltitude = false;
    bool showVerticalSpeed = false;
    double prevHeading;
    double prevSpeed;
    double prevAltitude;
    double prevVerticalSpeed;
    double setVerticalSpeed = 0;
    double setAltitude;
    bool managedHeading = true;
    bool managedSpeed = true;
    bool managedAltitude = true;

    // Hardware controls
    int speedControl = -1;
    int prevSpeedVal = 0;
    int prevSpeedPush = 0;
    int speedSetSel = 0;
    int altitudeSetSel = 0;
    time_t lastSpeedAdjust = 0;
    time_t lastSpeedPush = 0;
    time_t now;

public:
    autopilot();
    void render();
    void update();

private:
    void addGpio();
    void updateGpio();
    void machSwap();
    void toggleFlightDirector();
    void manSelSpeed();
    void manSelHeading();
    void manSelAltitude();
    void captureCurrent();
    void captureVerticalSpeed();
    void restoreVerticalSpeed();
    int adjustSpeed(int val, int adjust);
    double adjustMach(double val, int adjust);
    int adjustHeading(int val, int adjust);
    int adjustAltitude(int val, int adjust);
    int adjustVerticalSpeed(int val, int adjust);
};

#endif // _AUTOPILOT_H
