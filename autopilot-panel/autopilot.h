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
    Aircraft loadedAircraft = NO_AIRCRAFT;
    sevensegment* sevenSegment;

    AutopilotSpd autopilotSpd;
    AutopilotHdg autopilotHdg;
    AutopilotAlt autopilotAlt = AltHold;
    bool showMach = false;
    bool showSpeed = false;
    bool showHeading = false;
    bool showAltitude = false;
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
    int prevSpeedVal = 0;
    int prevSpeedPush = 0;
    int speedSetSel = 0;
    int altitudeSetSel = 0;
    time_t lastSpeedAdjust = 0;
    time_t lastSpeedPush = 0;
    time_t lastHeadingAdjust = 0;
    time_t lastHeadingPush = 0;
    time_t lastAltitudeAdjust = 0;
    time_t lastAltitudePush = 0;
    time_t lastVerticalSpeedAdjust = 0;
    time_t lastVerticalSpeedPush = 0;
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
    int adjustSpeed(int adjust);
    double adjustMach(int adjust);
    int adjustHeading(int adjust);
    int adjustAltitude(int adjust);
    int adjustVerticalSpeed(int adjust);
};

#endif // _AUTOPILOT_H
