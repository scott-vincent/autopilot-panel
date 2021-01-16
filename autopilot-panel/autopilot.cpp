#include <stdio.h>
#include <stdlib.h>
#include "gpioctrl.h"
#include "autopilot.h"

autopilot::autopilot()
{
    simVars = &globals.simVars->simVars;
    addGpio();

    // Initialise 7-segment displays

}

void autopilot::render()
{
    // Write to 7-segment displays

}

void autopilot::update()
{
    // Check for aircraft change
    bool aircraftChanged = (loadedAircraft != globals.aircraft);
    if (aircraftChanged) {
        loadedAircraft = globals.aircraft;
        showMach = false;
        showHeading = false;
        showSpeed = false;
        showAltitude = false;
        showVerticalSpeed = false;
        prevHeading = simVars->autopilotHeading;
        prevSpeed = simVars->autopilotAirspeed;
        prevAltitude = simVars->autopilotAltitude;
        prevVerticalSpeed = simVars->autopilotVerticalSpeed;
        setVerticalSpeed = 0;
        managedHeading = true;
        managedSpeed = true;
        managedAltitude = true;
    }

    updateGpio();

    // Show values if they get adjusted in sim
    if (showHeading == false && simVars->autopilotHeading != prevHeading) {
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
    globals.gpioCtrl->addSwitch("Autopilot");
}

void autopilot::updateGpio()
{
    // Speed rotate
    int val = globals.gpioCtrl->readRotation(speedControl);
    if (val != INT_MIN) {
        int diff = (val - prevSpeedVal) / 2;
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
                double newVal = adjustMach(simVars->autopilotMach, adjust);
                globals.simVars->write(KEY_AP_MACH_VAR_SET, newVal);
            }
            else {
                double newVal = adjustSpeed(simVars->autopilotAirspeed, adjust);
                globals.simVars->write(KEY_AP_SPD_VAR_SET, newVal);
            }
            prevSpeedVal = val;
        }
        time(&lastSpeedAdjust);
    }
    else if (lastSpeedAdjust != 0) {
        // Reset digit set selection if more than 5 seconds since last adjustment
        time(&now);
        if (now - lastSpeedAdjust > 5) {
            speedSetSel = 0;
            lastSpeedAdjust = 0;
        }
    }

    // Speed push
    val = globals.gpioCtrl->readPush(speedControl);
    if (val != INT_MIN) {
        // If previous state was unpressed then must have been pressed
        if (prevSpeedPush % 2 == 1) {
            if (speedSetSel == 1) {
                speedSetSel = 0;
            }
            else {
                speedSetSel++;
            }
            time(&lastSpeedPush);
        }
        else {
            // Released
            lastSpeedPush = 0;
        }
        prevSpeedPush = val;
    }

    // Long speed push (over 1 sec)
    if (lastSpeedPush > 0) {
        time(&now);
        if (now - lastSpeedPush > 1) {
            // Long press on speed switches between managed and selected
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
            lastSpeedPush = 0;
        }
    }


    /*
        // Adjust heading
        showHeading = true;
        double newVal = adjustHeading(simVars->autopilotHeading, adjust);
        globals.simVars->write(KEY_HEADING_BUG_SET, newVal);

        // Adjust altitude
        showAltitude = true;
        double newVal = adjustAltitude(simVars->autopilotAltitude, adjust);
        globals.simVars->write(KEY_AP_ALT_VAR_SET_ENGLISH, newVal);
        if (setVerticalSpeed != 0) {
            setAltitude = newVal;
        }

        // Adjust VerticalSpeed:
        showVerticalSpeed = true;
        double newVal = adjustVerticalSpeed(simVars->autopilotVerticalSpeed, adjust);
        globals.simVars->write(KEY_AP_VS_VAR_SET_ENGLISH, newVal);
        if (setVerticalSpeed != 0) {
            setVerticalSpeed = newVal;
        }
    */
}
/// <summary>
/// Switch speed display between knots and mach
/// </summary>
void autopilot::machSwap()
{
    // Set to current speed before switching
    if (showMach) {
        globals.simVars->write(KEY_AP_SPD_VAR_SET, simVars->asiAirspeed);
        showMach = false;
    }
    else {
        // For some weird reason you have to set mach * 100 !
        globals.simVars->write(KEY_AP_MACH_VAR_SET, simVars->asiMachSpeed * 100);
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

int autopilot::adjustSpeed(int val, int adjust)
{
    if (speedSetSel == 0) {
        // Adjust fives
        val += adjust * 5;
    }
    else {
        val += adjust;
    }

    if (val < 0) {
        val = 0;
    }
    else if (val > 990) {
        val = 990;
    }

    return val;
}

double autopilot::adjustMach(double val, int adjust)
{
    // For some reason you have to set mach * 100
    int machX100 = val * 100 + 0.5;
    machX100 += adjust;

    if (machX100 < 0) {
        machX100 = 0;
    }

    return machX100;
}

int autopilot::adjustHeading(int val, int adjust)
{
    val += adjust;
    if (val > 359) {
        val -= 360;
    }
    else if (val < 0) {
        val += 360;
    }

    return val;
}

int autopilot::adjustAltitude(int val, int adjust)
{
    int prevVal = val;

    if (altitudeSetSel == 0) {
        // Adjust thousands
        val += adjust * 1000;

        if (val < -9900) {
            val += 1000;
        }
    }
    else {
        // Adjust thousands and hundreds
        val += adjust * 100;

        if (val < -9900) {
            val += 100;
        }
    }

    if (autopilotAlt == VerticalSpeedHold) {
        // Cancel vertical speed hold when target altitude reached
        int diff = abs(val - simVars->altAltitude);
        if (diff < 210 || (val < simVars->altAltitude && prevVal > simVars->altAltitude)
            || (val > simVars->altAltitude && prevVal < simVars->altAltitude)) {
            autopilotAlt = AltHold;
        }
    }

    return val;
}

int autopilot::adjustVerticalSpeed(int val, int adjust)
{
    // Allow vertical speed to go negative
    val += adjust * 100;

    return val;
}
