#include <stdio.h>
#include <stdlib.h>
#include "gpioctrl.h"
#include "autopilot.h"

autopilot::autopilot()
{
    simVars = &globals.simVars->simVars;
    addGpio();

    // Initialise 7-segment displays
    sevenSegment = new sevensegment(0);
}

void autopilot::render()
{
    unsigned char display1[8];
    unsigned char display2[8];
    unsigned char display3[8];

    if (!globals.avionics) {
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
    }

    // Write to 7-segment displays
    if (showSpeed) {
        if (showMach) {
            int whole = machX100 / 100;
            int frac = machX100 % 100;
            sevenSegment->getSegData(display1, 1, whole, 1);
            sevenSegment->decimalSegData(display1, 0);
            sevenSegment->getSegData(&display1[1], 2, frac, 2);
        }
        else {
            sevenSegment->getSegData(display1, 3, speed, 3);
        }
    }
    else {
        sevenSegment->blankSegData(display1, 3, false);
    }

    if (managedSpeed) {
        sevenSegment->decimalSegData(display1, 2);
    }

    // Blank
    sevenSegment->blankSegData(&display1[3], 2, false);

    // Heading
    if (showHeading) {
        sevenSegment->getSegData(&display1[5], 3, heading, 3);
    }
    else {
        sevenSegment->blankSegData(&display1[5], 3, false);
    }

    if (managedHeading) {
        sevenSegment->decimalSegData(display1, 7);
    }

    // Altitude
    if (showAltitude) {
        sevenSegment->getSegData(display2, 8, altitude, 5);
    }
    else {
        sevenSegment->blankSegData(display2, 8, false);
    }

    if (managedAltitude) {
        sevenSegment->decimalSegData(display2, 7);
    }

    // Vertical speed
    if (showVerticalSpeed) {
        sevenSegment->getSegData(display3, 8, verticalSpeed, 4);
    }
    else {
        sevenSegment->blankSegData(display3, 8, false);
    }

    sevenSegment->writeSegData3(display1, display2, display3);

    // Write LEDs
    globals.gpioCtrl->writeLed(autopilotControl, simVars->autopilotEngaged);
    globals.gpioCtrl->writeLed(flightDirectorControl, simVars->flightDirectorActive);
    globals.gpioCtrl->writeLed(autothrottleControl, simVars->autothrottleActive);
    globals.gpioCtrl->writeLed(localiserControl, simVars->autopilotApproachHold);
    globals.gpioCtrl->writeLed(approachControl, simVars->autopilotGlideslopeHold);
}

void autopilot::update()
{
    // Check for aircraft change
    bool aircraftChanged = (loadedAircraft != globals.aircraft);
    if (aircraftChanged) {
        loadedAircraft = globals.aircraft;
        showMach = false;
        showSpeed = false;
        showHeading = false;
        showAltitude = true;
        showVerticalSpeed = false;
        machX100 = simVars->autopilotMach;
        speed = simVars->autopilotAirspeed;
        heading = simVars->autopilotHeading;
        altitude = simVars->autopilotAltitude;
        verticalSpeed = simVars->autopilotVerticalSpeed;
        prevSpeed = speed;
        prevHeading = heading;
        prevAltitude = altitude;
        prevVerticalSpeed = verticalSpeed;
        setVerticalSpeed = 0;
        managedSpeed = true;
        managedHeading = true;
        managedAltitude = true;
    }

    gpioSpeedInput();
    gpioHeadingInput();
    gpioAltitudeInput();
    gpioVerticalSpeedInput();
    gpioButtonsInput();

    // Show values if they get adjusted in sim
    if (showHeading == false && heading != prevHeading) {
        showHeading = true;
    }

    if (showSpeed == false && simVars->autopilotAirspeed != prevSpeed) {
        showSpeed = true;
    }

    if (showAltitude == false && simVars->autopilotAltitude != prevAltitude) {
        showAltitude = true;
    }

    if (showVerticalSpeed == false && simVars->autopilotVerticalSpeed != prevVerticalSpeed) {
        showVerticalSpeed = true;
    }

    // Only update local values from sim if they are not currently being
    // adjusted by the rotary encoders. This stops the displayed values
    // from jumping around due to lag of fetch/update cycle.
    if (lastSpdAdjust == 0) {
        machX100 = simVars->autopilotMach * 100;
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
    if (managedAltitude && setVerticalSpeed != 0 && autopilotAlt != AltHold) {
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
        int diff = (val - prevSpdVal) / 2;
        int adjust = 0;
        if (diff > 0) {
            adjust = 1;
        }
        else if (diff < 0) {
            adjust = -1;
        }

        if (adjust != 0) {
            // Adjust speed
            showSpeed = true;
            if (showMach) {
                double newVal = adjustMach(adjust);
                globals.simVars->write(KEY_AP_MACH_VAR_SET, newVal);
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
        time(&now);
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
            if (spdSetSel == 1) {
                spdSetSel = 0;
            }
            else {
                spdSetSel++;
            }
            time(&lastSpdPush);
        }
        else {
            // Released
            lastSpdPush = 0;
        }
        prevSpdPush = val;
    }

    // Speed long push (over 1 sec)
    if (lastSpdPush > 0) {
        time(&now);
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
            lastSpdPush = 0;
        }
    }
}

void autopilot::gpioHeadingInput()
{
    // Heading rotate
    int val = globals.gpioCtrl->readRotation(headingControl);
    if (val != INT_MIN) {
        int diff = (val - prevHdgVal) / 2;
        int adjust = 0;
        if (diff > 0) {
            adjust = 1;
        }
        else if (diff < 0) {
            adjust = -1;
        }

        if (adjust != 0) {
            // Adjust heading
            showHeading = true;
            double newVal = adjustHeading(adjust);
            globals.simVars->write(KEY_HEADING_BUG_SET, newVal);
            prevHdgVal = val;
        }
        time(&lastHdgAdjust);
    }
    else if (lastHdgAdjust != 0) {
        time(&now);
        if (now - lastHdgAdjust > 1) {
            lastHdgAdjust = 0;
        }
    }

    // Heading push
    val = globals.gpioCtrl->readPush(headingControl);
    if (val != INT_MIN) {
        if (prevHdgPush % 2 == 1) {
            // Short press switches between managed and selected
            if (autopilotHdg == HdgSet) {
                autopilotHdg = LevelFlight;
                globals.simVars->write(KEY_AP_HDG_HOLD_OFF);
                // Keep heading bug setting when heading hold turned off
                globals.simVars->write(KEY_HEADING_BUG_SET, simVars->autopilotHeading);
                manSelHeading();
            }
            else {
                autopilotHdg = HdgSet;
                globals.simVars->write(KEY_AP_HDG_HOLD_ON);
            }
        }
        prevHdgPush = val;
    }
}

void autopilot::gpioAltitudeInput()
{
    // Altitude rotate
    int val = globals.gpioCtrl->readRotation(altitudeControl);
    if (val != INT_MIN) {
        int diff = (val - prevAltVal) / 2;
        int adjust = 0;
        if (diff > 0) {
            adjust = 1;
        }
        else if (diff < 0) {
            adjust = -1;
        }

        if (adjust != 0) {
            // Adjust altitude
            showAltitude = true;
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
        time(&now);
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
        else {
            // Released
            lastAltPush = 0;
        }
        prevAltPush = val;
    }

    // Altitude long push (over 1 sec)
    if (lastAltPush > 0) {
        time(&now);
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
            lastAltPush = 0;
        }
    }
}

void autopilot::gpioVerticalSpeedInput()
{
    // Vertical speed rotate
    int val = globals.gpioCtrl->readRotation(verticalSpeedControl);
    if (val != INT_MIN) {
        int diff = (val - prevVsVal) / 2;
        int adjust = 0;
        if (diff > 0) {
            adjust = 1;
        }
        else if (diff < 0) {
            adjust = -1;
        }

        if (adjust != 0) {
            // Adjust vertical speed:
            showVerticalSpeed = true;
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
        time(&now);
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
            // Capture current values if autopilot is about to be engaged
            if (!simVars->autopilotEngaged) {
                captureCurrent();
            }
            globals.simVars->write(KEY_AP_MASTER);
        }
        prevApPush = val;
    }

    // Flight Director push
    val = globals.gpioCtrl->readPush(flightDirectorControl);
    if (val != INT_MIN) {
        // If previous state was unpressed then must have been pressed
        if (prevFdPush % 2 == 1) {
            toggleFlightDirector();
        }
        prevFdPush = val;
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
            // Toggle auto throttle
            globals.simVars->write(KEY_AUTO_THROTTLE_ARM);
        }
        prevAthrPush = val;
    }

    // Localiser push
    val = globals.gpioCtrl->readPush(localiserControl);
    if (val != INT_MIN) {
        if (prevLocPush % 2 == 1) {
            if (simVars->autopilotGlideslopeHold) {
                globals.simVars->write(KEY_AP_APR_HOLD_OFF);
            }
            globals.simVars->write(KEY_AP_LOC_HOLD);
        }
        prevLocPush = val;
    }

    // Approach push
    val = globals.gpioCtrl->readPush(approachControl);
    if (val != INT_MIN) {
        if (prevApprPush % 2 == 1) {
            if (simVars->autopilotGlideslopeHold) {
                globals.simVars->write(KEY_AP_APR_HOLD_OFF);
            }
            else {
                globals.simVars->write(KEY_AP_APR_HOLD_ON);
            }
        }
        prevApprPush = val;
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
        machX100 = simVars->asiMachSpeed * 100;
        // For some weird reason you have to set mach * 100 !
        globals.simVars->write(KEY_AP_MACH_VAR_SET, machX100);
        showMach = true;
    }
}

/// <summary>
/// If flight director is enabled/disabled just after
/// take off initialise autopilot settings.
/// </summary>
void autopilot::toggleFlightDirector()
{
    bool turnedOn = !(simVars->flightDirectorActive);
    globals.simVars->write(KEY_TOGGLE_FLIGHT_DIRECTOR);

    // Adjust autopilot settings if just after take off
    if (simVars->altAltitude > 1900) {
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

    if (turnedOn) {
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

    showHeading = true;
    showSpeed = true;
    showAltitude = true;
    showVerticalSpeed = true;
}

/// <summary>
/// Switch between managed and selected speed.
/// This is undocumented but works for the Airbus A320 neo.
/// </summary>
void autopilot::manSelSpeed()
{
    if (!simVars->flightDirectorActive) {
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
    if (!simVars->flightDirectorActive) {
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

    if (!showSpeed) {
        showSpeed = true;

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

    if (!showHeading) {
        showHeading = true;
    }

    globals.simVars->write(KEY_HEADING_BUG_SET, simVars->hiHeading);

    if (!showAltitude) {
        showAltitude = true;

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

    if (!showVerticalSpeed) {
        showVerticalSpeed = true;

        // Make sure VS is in correct direction when first shown
        if (setVerticalSpeed <= 0 && simVars->altAltitude < simVars->autopilotAltitude) {
            setVerticalSpeed = 1000;
        }
        else if (setVerticalSpeed >= 0 && simVars->altAltitude > simVars->autopilotAltitude) {
            setVerticalSpeed = -1000;
        }

        globals.simVars->write(KEY_AP_VS_VAR_SET_ENGLISH, setVerticalSpeed);
    }
}

void autopilot::restoreVerticalSpeed()
{
    // Ignore if autopilot disabled or altitude already reached
    if (!simVars->autopilotEngaged ||
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
    // For some reason you have to set mach * 100
    machX100 += adjust;

    if (machX100 < 0) {
        machX100 = 0;
    }

    return machX100;
}

int autopilot::adjustHeading(int adjust)
{
    heading += adjust;
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
    // Allow vertical speed to go negative
    verticalSpeed += adjust * 100;

    return verticalSpeed;
}
