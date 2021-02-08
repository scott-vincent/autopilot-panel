#include <stdio.h>
#include <stdlib.h>
#include "gpioctrl.h"
#include "autopilot.h"

autopilot::autopilot()
{
    simVars = &globals.simVars->simVars;
    addGpio();

    // Initialise 7-segment displays
    sevenSegment = new sevensegment(false, 0);

    fflush(stdout);
}

void autopilot::render()
{
    if (!globals.electrics) {
        // Turn off 7-segment displays
        sevenSegment->blankSegData(display1, 8, false);
        sevenSegment->blankSegData(display2, 8, false);
        sevenSegment->blankSegData(display3, 8, false);
        sevenSegment->writeSegData3(display1, display2, display3);

        // Turn off LEDS
        globals.gpioCtrl->writeLed(autopilotControl, false);
        globals.gpioCtrl->writeLed(flightDirectorControl, false);
        globals.gpioCtrl->writeLed(autothrottleControl, false);
        globals.gpioCtrl->writeLed(localiserControl, false);
        globals.gpioCtrl->writeLed(approachControl, false);

        // Make sure settings get re-initialised
        loadedAircraft = UNDEFINED;

        return;
    }

    // Write to 7-segment displays
    if (managedSpeed) {
        sevenSegment->blankSegData(display1, 3, true);
        sevenSegment->decimalSegData(display1, 2);
    }
    else if (showMach) {
        int machX100 = (mach + 0.005) * 100;
        int whole = machX100 / 100;
        int frac = machX100 % 100;
        sevenSegment->getSegData(display1, 1, whole, 1);
        sevenSegment->decimalSegData(display1, 0);
        sevenSegment->getSegData(&display1[1], 2, frac, 2);
    }
    else {
        sevenSegment->getSegData(display1, 3, speed + 0.5, 3);
        if (spdSetSel == 1) {
            sevenSegment->decimalSegData(display1, 1);
        }
    }

    // Blank
    sevenSegment->blankSegData(&display1[3], 2, false);

    // Heading
    if (managedHeading) {
        sevenSegment->blankSegData(&display1[5], 3, true);
        sevenSegment->decimalSegData(&display1[5], 2);
    }
    else {
        sevenSegment->getSegData(&display1[5], 3, heading, 3);
        if (hdgSetSel == 1) {
            sevenSegment->decimalSegData(&display1[5], 1);
        }
    }

    // Altitude
    sevenSegment->getSegData(display2, 8, altitude, 5);
    if (altSetSel == 1) {
        sevenSegment->decimalSegData(display2, 4);
    }
    if (managedAltitude) {
        sevenSegment->decimalSegData(display2, 7);
    }

    // Vertical speed
    if (autopilotAlt == VerticalSpeedHold) {
        sevenSegment->getSegData(display3, 8, verticalSpeed, 4);
    }
    else {
        sevenSegment->blankSegData(display3, 3, false);
        sevenSegment->blankSegData(&display3[3], 5, true);
    }

    sevenSegment->writeSegData3(display1, display2, display3);

    // Write LEDs
    globals.gpioCtrl->writeLed(autopilotControl, apEnabled);
    globals.gpioCtrl->writeLed(flightDirectorControl, fdEnabled);
    globals.gpioCtrl->writeLed(autothrottleControl, athrEnabled);
    globals.gpioCtrl->writeLed(localiserControl, locEnabled);
    globals.gpioCtrl->writeLed(approachControl, apprEnabled);
}

void autopilot::update()
{
    // Check for aircraft change
    bool aircraftChanged = (loadedAircraft != globals.aircraft);
    if (aircraftChanged) {
        loadedAircraft = globals.aircraft;
        airliner = (loadedAircraft != NO_AIRCRAFT && simVars->cruiseSpeed >= 300);
        showMach = false;
        mach = simVars->autopilotMach;
        speed = simVars->autopilotAirspeed;
        heading = simVars->autopilotHeading;
        altitude = simVars->autopilotAltitude;
        verticalSpeed = simVars->autopilotVerticalSpeed;
        apEnabled = simVars->autopilotEngaged;
        fdEnabled = simVars->flightDirectorActive;
        athrEnabled = simVars->autothrottleActive;
        locEnabled = simVars->autopilotApproachHold;
        apprEnabled = simVars->autopilotGlideslopeHold;
        setVerticalSpeed = 0;
        if (airliner) {
            if (simVars->asiAirspeed < 100) {
                managedSpeed = true;
            }
            managedHeading = true;
            managedAltitude = true;
        }
    }

    time(&now);
    gpioSpeedInput();
    gpioHeadingInput();
    gpioAltitudeInput();
    gpioVerticalSpeedInput();
    gpioButtonsInput();

    // Only update local values from sim if they are not currently being
    // adjusted by the rotary encoders. This stops the displayed values
    // from jumping around due to lag of fetch/update cycle.
    if (lastSpdAdjust == 0) {
        mach = simVars->autopilotMach;
        speed = simVars->autopilotAirspeed;
    }
    if (lastHdgAdjust == 0) {
        heading = simVars->autopilotHeading;
    }
    if (lastAltAdjust == 0) {
        altitude = simVars->autopilotAltitude;
    }
    if (lastVsAdjust == 0) {
        verticalSpeed = simVars->autopilotVerticalSpeed;
    }
    if (lastApAdjust == 0) {
        apEnabled = simVars->autopilotEngaged;
    }
    if (lastFdAdjust == 0) {
        fdEnabled = simVars->flightDirectorActive;
    }
    if (lastAthrAdjust == 0) {
        athrEnabled = simVars->autothrottleActive;
    }
    if (lastLocAdjust == 0) {
        locEnabled = simVars->autopilotApproachHold;
    }
    if (lastApprAdjust == 0) {
        apprEnabled = simVars->autopilotGlideslopeHold;
    }

    if (simVars->autopilotAirspeedHold == 1) {
        autopilotSpd = SpdHold;
    }
    else {
        autopilotSpd = NoSpd;
    }

    if (simVars->autopilotHeadingLock == 1) {
        autopilotHdg = HdgSet;
    }
    else if (simVars->autopilotLevel == 1) {
        autopilotHdg = LevelFlight;
    }
    else {
        autopilotHdg = NoHdg;
    }

    if (simVars->autopilotAltLock == 1) {
        if (autopilotAlt == VerticalSpeedHold) {
            // Revert to alt hold when within range of target altitude
            int diff = abs(simVars->altAltitude - simVars->autopilotAltitude);
            if (diff < 210) {
                autopilotAlt = AltHold;
            }
        }
        else {
            autopilotAlt = AltHold;
        }
    }
    else if (simVars->autopilotVerticalHold == 1) {
        autopilotAlt = VerticalSpeedHold;
    }
    else if (simVars->autopilotPitchHold == 1) {
        autopilotAlt = PitchHold;
    }
    else {
        autopilotAlt = NoAlt;
    }

    // Alt hold can disengage when passing a waypoint so
    // re-enable and set previous altitude / vertical speed.
    if (managedAltitude && !apprEnabled && setVerticalSpeed != 0 && autopilotAlt != AltHold) {
        restoreVerticalSpeed();
    }
}

void autopilot::addGpio()
{
    speedControl = globals.gpioCtrl->addRotaryEncoder("Speed");
    headingControl = globals.gpioCtrl->addRotaryEncoder("Heading");
    altitudeControl = globals.gpioCtrl->addRotaryEncoder("Altitude");
    verticalSpeedControl = globals.gpioCtrl->addRotaryEncoder("Vertical Speed");
    autopilotControl = globals.gpioCtrl->addButton("Autopilot");
    flightDirectorControl = globals.gpioCtrl->addButton("Flight Director");
    machControl = globals.gpioCtrl->addButton("Mach");
    autothrottleControl = globals.gpioCtrl->addButton("Autothrottle");
    localiserControl = globals.gpioCtrl->addButton("Localiser");
    approachControl = globals.gpioCtrl->addButton("Approach");
}

void autopilot::gpioSpeedInput()
{
    // Speed rotate
    int val = globals.gpioCtrl->readRotation(speedControl);
    if (val != INT_MIN) {
        int diff = (val - prevSpdVal) / 4;
        int adjust = 0;
        if (diff > 0) {
            adjust = 1;
        }
        else if (diff < 0) {
            adjust = -1;
        }

        if (adjust != 0) {
            // Adjust speed
            if (showMach) {
                double newVal = adjustMach(adjust);
                globals.simVars->write(KEY_AP_MACH_VAR_SET, newVal * 100);
            }
            else {
                double newVal = adjustSpeed(adjust);
                globals.simVars->write(KEY_AP_SPD_VAR_SET, newVal);
            }
            prevSpdVal = val;
        }
        time(&lastSpdAdjust);
    }
    else if (lastSpdAdjust != 0) {
        // Reset digit set selection if more than 2 seconds since last adjustment
        if (now - lastSpdAdjust > 2) {
            spdSetSel = 0;
            lastSpdAdjust = 0;
        }
    }

    // Speed push
    val = globals.gpioCtrl->readPush(speedControl);
    if (val != INT_MIN) {
        if (prevSpdPush % 2 == 1) {
            // Short press switches between 5 knot and 1 knot increments
            // Default is 5 knots
            if (spdSetSel == 0) {
                spdSetSel = 1;
            }
            else {
                spdSetSel = 0;
            }
            time(&lastSpdPush);
        }
        if (val % 2 == 1) {
            // Released
            lastSpdPush = 0;
        }
        prevSpdPush = val;
    }

    // Speed long push (over 1 sec)
    if (lastSpdPush > 0) {
        if (now - lastSpdPush > 1) {
            // Long press switches between managed and selected
            if (autopilotSpd == SpdHold) {
                globals.simVars->write(KEY_AP_MACH_OFF);
                globals.simVars->write(KEY_AP_AIRSPEED_OFF);
            }
            else {
                if (showMach) {
                    globals.simVars->write(KEY_AP_MACH_ON);
                }
                else {
                    globals.simVars->write(KEY_AP_AIRSPEED_ON);
                }
            }
            manSelSpeed();
            spdSetSel = 0;
            lastSpdPush = 0;
        }
    }
}

void autopilot::gpioHeadingInput()
{
    // Heading rotate
    int val = globals.gpioCtrl->readRotation(headingControl);
    if (val != INT_MIN) {
        int diff = (val - prevHdgVal) / 4;
        int adjust = 0;
        if (diff > 0) {
            adjust = 1;
        }
        else if (diff < 0) {
            adjust = -1;
        }

        if (adjust != 0) {
            // Adjust heading
            double newVal = adjustHeading(adjust);
            globals.simVars->write(KEY_HEADING_BUG_SET, newVal);
            prevHdgVal = val;
        }
        time(&lastHdgAdjust);
    }
    else if (lastHdgAdjust != 0) {
        if (now - lastHdgAdjust > 2) {
            hdgSetSel = 0;
            lastHdgAdjust = 0;
        }
    }

    // Heading push
    val = globals.gpioCtrl->readPush(headingControl);
    if (val != INT_MIN) {
        if (prevHdgPush % 2 == 1) {
            // Short press switches between 1 degree and 5 degree increments
            // Default is 1 degree
            if (hdgSetSel == 0) {
                hdgSetSel = 1;
            }
            else {
                hdgSetSel = 0;
            }
            time(&lastHdgPush);
        }
        if (val % 2 == 1) {
            // Released
            lastHdgPush = 0;
        }
        prevHdgPush = val;
    }

    // Heading long push (over 1 sec)
    if (lastHdgPush > 0) {
        if (now - lastHdgPush > 1) {
            // Long press switches between managed and selected when flight
            // director is active or toggles heading hold when it isn't.
            if (fdEnabled) {
                if (managedHeading) {
                    // Keep heading bug setting when managed mode turned off
                    globals.simVars->write(KEY_HEADING_BUG_SET, simVars->autopilotHeading);
                }
            }
            else if (autopilotHdg == HdgSet) {
                autopilotHdg = LevelFlight;
                globals.simVars->write(KEY_AP_HDG_HOLD_OFF);
                // Keep heading bug setting when heading hold turned off
                globals.simVars->write(KEY_HEADING_BUG_SET, simVars->autopilotHeading);
            }
            else {
                autopilotHdg = HdgSet;
                globals.simVars->write(KEY_AP_HDG_HOLD_ON);
            }
            manSelHeading();
            hdgSetSel = 0;
            lastHdgPush = 0;
        }
    }
}

void autopilot::gpioAltitudeInput()
{
    // Altitude rotate
    int val = globals.gpioCtrl->readRotation(altitudeControl);
    if (val != INT_MIN) {
        int diff = (val - prevAltVal) / 4;
        int adjust = 0;
        if (diff > 0) {
            adjust = 1;
        }
        else if (diff < 0) {
            adjust = -1;
        }

        if (adjust != 0) {
            // Adjust altitude
            double newVal = adjustAltitude(adjust);
            globals.simVars->write(KEY_AP_ALT_VAR_SET_ENGLISH, newVal);
            if (setVerticalSpeed != 0) {
                setAltitude = newVal;
            }
            prevAltVal = val;
        }
        time(&lastAltAdjust);
    }
    else if (lastAltAdjust != 0) {
        // Reset digit set selection if more than 2 seconds since last adjustment
        if (now - lastAltAdjust > 2) {
            altSetSel = 0;
            lastAltAdjust = 0;
        }
    }

    // Altitude push
    val = globals.gpioCtrl->readPush(altitudeControl);
    if (val != INT_MIN) {
        if (prevAltPush % 2 == 1) {
            // Short press switches between 1000ft and 100ft increments
            if (altSetSel == 1) {
                altSetSel = 0;
            }
            else {
                altSetSel++;
            }
            time(&lastAltPush);
        }
        if (val % 2 == 1) {
            // Released
            lastAltPush = 0;
        }
        prevAltPush = val;
    }

    // Altitude long push (over 1 sec)
    if (lastAltPush > 0) {
        if (now - lastAltPush > 1) {
            // Long press switches between managed and selected
            if (autopilotAlt == AltHold) {
                autopilotAlt = PitchHold;
                globals.simVars->write(KEY_AP_ALT_HOLD_OFF);
            }
            else {
                autopilotAlt = AltHold;
                globals.simVars->write(KEY_AP_ALT_HOLD_ON);
            }
            manSelAltitude();
            setVerticalSpeed = 0;
            altSetSel = 0;
            lastAltPush = 0;
        }
    }
}

void autopilot::gpioVerticalSpeedInput()
{
    // Vertical speed rotate
    int val = globals.gpioCtrl->readRotation(verticalSpeedControl);
    if (val != INT_MIN) {
        int diff = (val - prevVsVal) / 4;
        int adjust = 0;
        if (diff > 0) {
            adjust = 1;
        }
        else if (diff < 0) {
            adjust = -1;
        }

        if (adjust != 0) {
            // Adjust vertical speed:
            double newVal = adjustVerticalSpeed(adjust);
            globals.simVars->write(KEY_AP_VS_VAR_SET_ENGLISH, newVal);
            if (setVerticalSpeed != 0) {
                setVerticalSpeed = newVal;
            }
            prevVsVal = val;
        }
        time(&lastVsAdjust);
    }
    else if (lastVsAdjust != 0) {
        if (now - lastVsAdjust > 1) {
            lastVsAdjust = 0;
        }
    }

    // Vertical speed push
    val = globals.gpioCtrl->readPush(verticalSpeedControl);
    if (val != INT_MIN) {
        // Short press switches between managed and selected
        if (prevVsPush % 2 == 1) {
            autopilotAlt = VerticalSpeedHold;
            manSelAltitude();
            globals.simVars->write(KEY_AP_ALT_VAR_SET_ENGLISH, simVars->autopilotAltitude);
            globals.simVars->write(KEY_AP_ALT_HOLD_ON);
            manSelAltitude();
            captureVerticalSpeed();
        }
        prevVsPush = val;
    }
}

void autopilot::gpioButtonsInput()
{
    // Autopilot push
    int val = globals.gpioCtrl->readPush(autopilotControl);
    if (val != INT_MIN) {
        if (prevApPush % 2 == 1) {
            // Toggle autopilot
            apEnabled = !apEnabled;
            globals.gpioCtrl->writeLed(autopilotControl, apEnabled);

            // Capture current values if autopilot has been engaged
            if (apEnabled) {
                captureCurrent();
            }
            globals.simVars->write(KEY_AP_MASTER);
        }
        prevApPush = val;
        time(&lastApAdjust);
    }
    else if (lastApAdjust != 0) {
        if (now - lastApAdjust > 1) {
            lastApAdjust = 0;
        }
    }

    // Flight Director push
    val = globals.gpioCtrl->readPush(flightDirectorControl);
    if (val != INT_MIN) {
        if (prevFdPush % 2 == 1) {
            // Toggle flight director
            fdEnabled = !fdEnabled;
            globals.gpioCtrl->writeLed(flightDirectorControl, fdEnabled);

            toggleFlightDirector();
        }
        prevFdPush = val;
        time(&lastFdAdjust);
    }
    else if (lastFdAdjust != 0) {
        if (now - lastFdAdjust > 1) {
            lastFdAdjust = 0;
        }
    }

    // Mach push
    val = globals.gpioCtrl->readPush(machControl);
    if (val != INT_MIN) {
        if (prevMachPush % 2 == 1) {
            // Swap between knots and mach
            machSwap();
        }
        prevMachPush = val;
    }

    // Autothrottle push
    val = globals.gpioCtrl->readPush(autothrottleControl);
    if (val != INT_MIN) {
        if (prevAthrPush % 2 == 1) {
            // Toggle autothrottle
            athrEnabled = !athrEnabled;
            globals.gpioCtrl->writeLed(autothrottleControl, athrEnabled);

            globals.simVars->write(KEY_AUTO_THROTTLE_ARM);
        }
        prevAthrPush = val;
        time(&lastAthrAdjust);
    }
    else if (lastAthrAdjust != 0) {
        if (now - lastAthrAdjust > 1) {
            lastAthrAdjust = 0;
        }
    }

    // Localiser push
    val = globals.gpioCtrl->readPush(localiserControl);
    if (val != INT_MIN) {
        if (prevLocPush % 2 == 1) {
            // Toggle localiser
            locEnabled = !locEnabled;
            globals.gpioCtrl->writeLed(localiserControl, locEnabled);

            if (apprEnabled) {
                globals.simVars->write(KEY_AP_APR_HOLD_OFF);
            }
            globals.simVars->write(KEY_AP_LOC_HOLD);
        }
        prevLocPush = val;
        time(&lastLocAdjust);
    }
    else if (lastLocAdjust != 0) {
        if (now - lastLocAdjust > 1) {
            lastLocAdjust = 0;
        }
    }

    // Approach push
    val = globals.gpioCtrl->readPush(approachControl);
    if (val != INT_MIN) {
        if (prevApprPush % 2 == 1) {
            // Toggle approach
            apprEnabled = !apprEnabled;
            globals.gpioCtrl->writeLed(approachControl, apprEnabled);

            if (apprEnabled) {
                globals.simVars->write(KEY_AP_APR_HOLD_ON);
            }
            else {
                globals.simVars->write(KEY_AP_APR_HOLD_OFF);
            }
        }
        prevApprPush = val;
        time(&lastApprAdjust);
    }
    else if (lastApprAdjust != 0) {
        if (now - lastApprAdjust > 1) {
            lastApprAdjust = 0;
        }
    }
}

/// <summary>
/// Switch speed display between knots and mach
/// </summary>
void autopilot::machSwap()
{
    // Set to current speed before switching
    if (showMach) {
        speed = simVars->asiAirspeed;
        globals.simVars->write(KEY_AP_SPD_VAR_SET, speed);
        showMach = false;
    }
    else {
        mach = simVars->asiMachSpeed;
        // For some weird reason you have to set mach * 100 !
        globals.simVars->write(KEY_AP_MACH_VAR_SET, mach * 100);
        showMach = true;
    }
}

/// <summary>
/// If flight director is enabled/disabled just after
/// take off initialise autopilot settings.
/// </summary>
void autopilot::toggleFlightDirector()
{
    globals.simVars->write(KEY_TOGGLE_FLIGHT_DIRECTOR);

    // Adjust autopilot settings if just after take off
    if (simVars->altAltitude > 3000 || simVars->vsiVerticalSpeed < 1) {
        return;
    }

    // Initial settings
    int holdSpeed = 200;
    setAltitude = 4000;
    setVerticalSpeed = 1500;

    managedSpeed = false;
    globals.simVars->write(KEY_SPEED_SLOT_INDEX_SET, 1);
    managedAltitude = true;
    globals.simVars->write(KEY_ALTITUDE_SLOT_INDEX_SET, 2);

    if (fdEnabled) {
        // Use managed heading if FD turned on
        managedHeading = true;
        globals.simVars->write(KEY_HEADING_SLOT_INDEX_SET, 2);
    }
    else {
        // Use current heading if FD turned off
        managedHeading = false;
        globals.simVars->write(KEY_HEADING_SLOT_INDEX_SET, 1);
        globals.simVars->write(KEY_HEADING_BUG_SET, simVars->hiHeading);
    }

    globals.simVars->write(KEY_AP_SPD_VAR_SET, holdSpeed);
    globals.simVars->write(KEY_AP_ALT_HOLD_ON);
    globals.simVars->write(KEY_AP_ALT_VAR_SET_ENGLISH, setAltitude);
    globals.simVars->write(KEY_AP_AIRSPEED_ON);
    globals.simVars->write(KEY_AP_VS_VAR_SET_ENGLISH, setVerticalSpeed);
}

/// <summary>
/// Switch between managed and selected speed.
/// This is undocumented but works for the Airbus A320 neo.
/// </summary>
void autopilot::manSelSpeed()
{
    if (!airliner) {
        return;
    }

    if (!fdEnabled) {
        managedSpeed = false;
    }
    else {
        managedSpeed = !managedSpeed;
    }

    if (managedSpeed) {
        globals.simVars->write(KEY_SPEED_SLOT_INDEX_SET, 2);
    }
    else {
        globals.simVars->write(KEY_SPEED_SLOT_INDEX_SET, 1);
    }
}

/// <summary>
/// Switch between managed and selected heading.
/// This is undocumented but works for the Airbus A320 neo.
/// </summary>
void autopilot::manSelHeading()
{
    if (!airliner) {
        return;
    }

    if (!fdEnabled) {
        managedHeading = false;
    }
    else {
        managedHeading = !managedHeading;
    }

    if (managedHeading) {
        globals.simVars->write(KEY_HEADING_SLOT_INDEX_SET, 2);
    }
    else {
        globals.simVars->write(KEY_HEADING_SLOT_INDEX_SET, 1);
    }
}

/// <summary>
/// Switch between managed and selected altitude.
/// This is undocumented but works for the Airbus A320 neo.
/// </summary>
void autopilot::manSelAltitude()
{
    if (!airliner) {
        return;
    }

    managedAltitude = !managedAltitude;
    if (managedAltitude) {
        globals.simVars->write(KEY_ALTITUDE_SLOT_INDEX_SET, 2);
    }
    else {
        globals.simVars->write(KEY_ALTITUDE_SLOT_INDEX_SET, 1);
    }
}

void autopilot::captureCurrent()
{
    int holdSpeed = simVars->asiAirspeed;

    if (!managedSpeed) {
        // Set autopilot speed to within 5 knots of current speed
        int fives = holdSpeed % 5;
        if (fives < 3) {
            holdSpeed -= fives;
        }
        else {
            holdSpeed += 5 - fives;
        }
    }

    globals.simVars->write(KEY_AP_SPD_VAR_SET, holdSpeed);

    if (!managedHeading) {
        globals.simVars->write(KEY_HEADING_BUG_SET, simVars->hiHeading);
    }

    if (!managedAltitude) {
        // Set autopilot altitude to within 100ft of current altitude
        int holdAlt = simVars->altAltitude;
        int hundreds = holdAlt % 100;
        if (hundreds < 30) {
            holdAlt -= hundreds;
        }
        else {
            holdAlt += 100 - hundreds;
        }
        globals.simVars->write(KEY_AP_ALT_VAR_SET_ENGLISH, holdAlt);
    }
}

void autopilot::captureVerticalSpeed()
{
    setVerticalSpeed = simVars->autopilotVerticalSpeed;
    setAltitude = simVars->autopilotAltitude;
}

void autopilot::restoreVerticalSpeed()
{
    // Ignore if autopilot disabled or altitude already reached
    if (!apEnabled ||
        (setVerticalSpeed < 0 && simVars->altAltitude < simVars->autopilotAltitude) ||
        (setVerticalSpeed > 0 && simVars->altAltitude > simVars->autopilotAltitude)) {
        setVerticalSpeed = 0;
        return;
    }

    globals.simVars->write(KEY_AP_ALT_VAR_SET_ENGLISH, setAltitude);
    globals.simVars->write(KEY_AP_VS_VAR_SET_ENGLISH, setVerticalSpeed);
    globals.simVars->write(KEY_AP_ALT_HOLD_ON);
}

int autopilot::adjustSpeed(int adjust)
{
    if (spdSetSel == 0) {
        // Adjust fives
        speed += adjust * 5;
    }
    else {
        speed += adjust;
    }

    if (speed < 0) {
        speed = 0;
    }
    else if (speed > 990) {
        speed = 990;
    }

    return speed;
}

double autopilot::adjustMach(int adjust)
{
    mach += adjust / 100.0;

    if (mach < 0) {
        mach = 0;
    }

    return mach;
}

int autopilot::adjustHeading(int adjust)
{
    if (hdgSetSel == 0) {
        heading += adjust;
    }
    else {
        // Adjust fives
        heading += adjust * 5;
    }

    if (heading > 359) {
        heading -= 360;
    }
    else if (heading < 0) {
        heading += 360;
    }

    return heading;
}

int autopilot::adjustAltitude(int adjust)
{
    int prevVal = altitude;

    if (altSetSel == 0) {
        // Adjust thousands
        altitude += adjust * 1000;

        if (altitude < 100) {
            altitude = 100;
        }
        else if (altitude == 1100) {
            altitude = 1000;
        }
    }
    else {
        // Adjust thousands and hundreds
        altitude += adjust * 100;

        if (altitude < 100) {
            altitude = 100;
        }
    }

    if (autopilotAlt == VerticalSpeedHold) {
        // Cancel vertical speed hold when target altitude reached
        int diff = abs(altitude - simVars->altAltitude);
        if (diff < 210 || (altitude < simVars->altAltitude && prevVal > simVars->altAltitude)
            || (altitude > simVars->altAltitude && prevVal < simVars->altAltitude)) {
            autopilotAlt = AltHold;
        }
    }

    return altitude;
}

int autopilot::adjustVerticalSpeed(int adjust)
{
    // Can only adjust vertical speed when in vertical speed hold mode
    if (autopilotAlt == VerticalSpeedHold) {
        // Allow vertical speed to go negative
        verticalSpeed += adjust * 100;
    }

    return verticalSpeed;
}
