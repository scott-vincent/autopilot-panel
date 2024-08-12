#include <stdio.h>
#include <stdlib.h>
#include <math.h>
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
    if (managedSpeed || !airliner) {
        sevenSegment->blankSegData(display1, 3, true);
        if (airliner) {
            sevenSegment->decimalSegData(display1, 2);
        }
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
    if (managedHeading || (!airliner && autopilotHdg != HdgSet)) {
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
    if (loadedAircraft == AIRBUS_A310 && !managedAltitude) {
        // For A310, managedAltitude == Selected VS
        sevenSegment->blankSegData(display3, 3, false);
        sevenSegment->blankSegData(&display3[3], 5, true);
    }
    else if (autopilotAlt == VerticalSpeedHold) {
        sevenSegment->getSegData(display3, 8, verticalSpeed, 4);
    }
    else if (simVars->autopilotVerticalHold == -1) {
        // FPA mode so verticalSpeed is the flight path angle rather than V/S
        sevenSegment->getSegDegrees(display3, 8, fpaX10);
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
    bool aircraftChanged = (globals.electrics && loadedAircraft != globals.aircraft);
    if (aircraftChanged) {
        loadedAircraft = globals.aircraft;
        airliner = (loadedAircraft != NO_AIRCRAFT && simVars->cruiseSpeed >= 300);
        showMach = false;
        mach = simVars->autopilotMach;
        speed = simVars->autopilotAirspeed;
        heading = simVars->autopilotHeading;
        altitude = simVars->autopilotAltitude;
        if (simVars->autopilotVerticalHold == -1) {
            fpaX10 = simVars->autopilotVerticalSpeed * 10;
        }
        else {
            verticalSpeed = simVars->autopilotVerticalSpeed;
        }
        apEnabled = simVars->autopilotEngaged;
        fdEnabled = simVars->flightDirectorActive;
        athrEnabled = simVars->autothrottleActive;
        locEnabled = simVars->autopilotApproachHold;
        apprEnabled = simVars->autopilotGlideslopeHold;
        lastSetHeading = -1;
        setVerticalSpeed = 0;
        if (loadedAircraft != AIRBUS_A310 && loadedAircraft != FBW_A320 && loadedAircraft != BOEING_747) {
            managedSpeed = false;
            managedHeading = false;
            managedAltitude = false;
        }
        prevSpdValSb = simVars->sbEncoder[2];
        prevSpdPushSb = simVars->sbButton[2];
        prevHdgValSb = simVars->sbEncoder[3];
        prevHdgPushSb = simVars->sbButton[3];
        prevAltValSb = simVars->sbEncoder[1];
        prevAltPushSb = simVars->sbButton[1];
        prevVsValSb = simVars->sbEncoder[0];
        prevVsPushSb = simVars->sbButton[0];
        prevApPushSb = simVars->sbButton[6];
        prevLocPushSb = simVars->sbButton[5];
        prevApprPushSb = simVars->sbButton[4];
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
        if (loadedAircraft == AIRBUS_A310) {
            managedSpeed = simVars->jbManagedSpeed;
            showMach = simVars->jbShowMach != 0;
        }
        else if (loadedAircraft == FBW_A320) {
            managedSpeed = simVars->jbManagedSpeed;
            showMach = simVars->jbAutothrustMode == 8;
        }
    }
    if (orbit > 0) {
        continueOrbit();
    }
    if (lastHdgAdjust == 0) {
        heading = simVars->autopilotHeading;
        if (loadedAircraft == AIRBUS_A310 || loadedAircraft == FBW_A320) {
            managedHeading = simVars->jbManagedHeading;
        }

        // Hack - Fix broken heading (changes to -1 on its own!)
        if (!managedHeading && simVars->autopilotHeading == -1) {
            if (lastSetHeading != -1) {
                sendEvent(KEY_HEADING_BUG_SET, lastSetHeading);
                printf("Hack: Change heading back to %f\n", lastSetHeading);
                fflush(stdout);
            }
        }
        else {
            lastSetHeading = simVars->autopilotHeading;
        }
    }
    if (lastAltAdjust == 0) {
        altitude = simVars->autopilotAltitude;
        if (loadedAircraft == AIRBUS_A310 || loadedAircraft == FBW_A320) {
            managedAltitude = simVars->jbManagedAltitude;
        }
    }
    if (lastVsAdjust == 0) {
        if (simVars->autopilotVerticalHold == -1) {
            fpaX10 = simVars->autopilotVerticalSpeed * 10;
        }
        else {
            verticalSpeed = simVars->autopilotVerticalSpeed;
        }
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

    if (simVars->autopilotVerticalHold == 1 || (!airliner && simVars->autopilotVerticalSpeed != 0)) {
        autopilotAlt = VerticalSpeedHold;
    }
    else if (autopilotAlt == VerticalSpeedHold) {
        // Reached required altitude so revert to altitude hold
        autopilotAlt = AltHold;
    }
    else if (simVars->autopilotPitchHold == 1) {
        autopilotAlt = PitchHold;
    }

    // If pressing brake pedal cancel the autobrake
    if (simVars->jbAutobrake > 0 && (simVars->brakeLeftPedal > 5 || simVars->brakeRightPedal > 5)) {
        sendEvent(KEY_AUTOBRAKE, 0);
    }

    //  Fix autopilot ALT set
    if (altSetRetry > 0) {
        altSetRetry--;
        if (lastAltVal != -1 && simVars->autopilotAltitude != lastAltVal) {
            // Keep trying to set the correct value
            if (altSetRetry % 4 == 0) {
                int diff = lastAltVal - simVars->autopilotAltitude;
                if (diff > 900) {
                    sendEvent(KEY_AP_ALT_VAR_INC, 1000);
                }
                else if (diff < -900) {
                    sendEvent(KEY_AP_ALT_VAR_DEC, 1000);
                }
                else if (diff > 90) {
                    sendEvent(KEY_AP_ALT_VAR_INC, 100);
                }
                else if (diff < -90) {
                    sendEvent(KEY_AP_ALT_VAR_DEC, 100);
                }
            }
        }
    }

    // Fix autopilot VS set
    if (vsSetRetry > 0) {
        vsSetRetry--;
        if (lastVsVal != -1 && simVars->autopilotVerticalSpeed != lastVsVal) {
            // Keep trying to set the correct value (but not too quickly)
            if (vsSetRetry % 10 == 0) {
                int diff = lastVsVal - simVars->autopilotVerticalSpeed;
                if (diff > 900) {
                    sendEvent(KEY_AP_VS_VAR_INC, 1000);
                }
                else if (diff < -900) {
                    sendEvent(KEY_AP_VS_VAR_DEC, 1000);
                }
                else if (diff > 90) {
                    sendEvent(KEY_AP_VS_VAR_INC, 100);
                }
                else if (diff < -90) {
                    sendEvent(KEY_AP_VS_VAR_DEC, 100);
                }
            }
        }
    }
}

void autopilot::sendEvent(EVENT_ID id, double value = 0.0)
{
    if (loadedAircraft == FBW_A320) {
        // Convert events to A32NX specific events
        switch (id) {
        case KEY_AP_HDG_HOLD_OFF:
            id = A32NX_FCU_HDG_PUSH;
            break;
        case KEY_AP_HDG_HOLD_ON:
            id = A32NX_FCU_HDG_PULL;
            break;
        case KEY_AP_ALT_HOLD_OFF:
            id = A32NX_FCU_ALT_PUSH;
            break;
        case KEY_AP_ALT_HOLD_ON:
            id = A32NX_FCU_ALT_PULL;
            break;
        case KEY_AP_VS_VAR_SET_ENGLISH:
            if (value == 0) {
                globals.simVars->write(A32NX_FCU_VS_PUSH);
            }
            else {
                globals.simVars->write(A32NX_FCU_VS_PULL);
            }
            id = A32NX_FCU_VS_SET;
            break;
        case KEY_AP_APR_HOLD_OFF:
            // Don't toggle if already in required state
            if (simVars->jbApprMode == 0) return;
            id = A32NX_FCU_APPR_PUSH;
            break;
        case KEY_AP_APR_HOLD_ON:
            // Don't toggle if already in required state
            if (simVars->jbApprMode == 1) return;
            id = A32NX_FCU_APPR_PUSH;
            break;
        case KEY_HEADING_BUG_SET:
            globals.simVars->write(id, value);
            id = A32NX_FCU_HDG_SET;
            break;
        case KEY_AP_MACH_VAR_SET:
        case KEY_AP_SPD_VAR_SET:
            id = A32NX_FCU_SPD_SET;
            break;
        case KEY_AP_SPEED_SLOT_INDEX_SET:
            if (value == 2) {
                id = A32NX_FCU_SPD_PUSH;
            }
            else {
                id = A32NX_FCU_SPD_PULL;
            }
            break;
        case KEY_AP_HEADING_SLOT_INDEX_SET:
            if (value == 2) {
                id = A32NX_FCU_HDG_PUSH;
            }
            else {
                id = A32NX_FCU_HDG_PULL;
            }
            break;
        case KEY_AP_VS_SLOT_INDEX_SET:
            if (value == 2) {
                id = A32NX_FCU_ALT_PUSH;
            }
            else {
                id = A32NX_FCU_ALT_PULL;
            }
            break;
        }
    }

    globals.simVars->write(id, value);
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
    int diff = (val - prevSpdVal) / 4;
    bool switchBox = false;

    if (val != INT_MIN) {
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
                if (loadedAircraft == AIRBUS_A310) {
                    sendEvent(KEY_AP_SPD_VAR_SET, newVal);
                }
                else {
                    sendEvent(KEY_AP_MACH_VAR_SET, newVal * 100 + 0.5);
                }
            }
            else {
                double newVal = adjustSpeed(adjust);
                sendEvent(KEY_AP_SPD_VAR_SET, newVal);
            }
            if (switchBox) {
                prevSpdValSb = val;
            }
            else {
                prevSpdVal = val;
            }
        }
        time(&lastSpdAdjust);
    }
    else if (lastSpdAdjust != 0) {
        // Reset digit set selection if more than 3 seconds since last adjustment
        if (now - lastSpdAdjust > 3) {
            spdSetSel = 0;
            lastSpdAdjust = 0;
        }
    }

    // Speed push
    val = globals.gpioCtrl->readPush(speedControl);
    int prevVal = prevSpdPush;
    switchBox = false;

    if (val != INT_MIN) {
        if (prevVal % 2 == 1) {
            // Short press switches between 5 knot and 1 knot increments
            // Default is 5 knots
            if (spdSetSel == 0) {
                spdSetSel = 1;
            }
            else {
                spdSetSel = 0;
            }
            time(&lastSpdAdjust);
            time(&lastSpdPush);
        }
        if (val % 2 == 1) {
            // Released
            lastSpdPush = 0;
        }
        if (switchBox) {
            prevSpdPushSb = val;
        }
        else {
            prevSpdPush = val;
        }
    }

    // Speed long push (over 1 sec)
    if (lastSpdPush > 0) {
        if (now - lastSpdPush > 1) {
            // Long press switches between managed and selected
            if (autopilotSpd == SpdHold) {
                sendEvent(KEY_AP_MACH_OFF);
                sendEvent(KEY_AP_AIRSPEED_OFF);
            }
            else {
                if (showMach) {
                    sendEvent(KEY_AP_MACH_ON);
                }
                else {
                    sendEvent(KEY_AP_AIRSPEED_ON);
                }
            }
            manSelSpeed();
            spdSetSel = 0;
            lastSpdPush = 0;
            time(&lastSpdAdjust);
        }
    }
}

void autopilot::gpioHeadingInput()
{
    // Heading rotate
    int val = globals.gpioCtrl->readRotation(headingControl);
    int diff = (val - prevHdgVal) / 4;
    bool switchBox = false;

    // If mode is instruments but we are an airliner then first
    // switchbox knob does autopilot heading instead of heading bug.
    bool ignore = simVars->sbMode == 2 || (simVars->sbMode == 3 && !airliner);

    if (ignore) {
        prevHdgValSb = simVars->sbEncoder[3];
    }
    else if (simVars->sbEncoder[3] != prevHdgValSb) {
        val = simVars->sbEncoder[3];
        diff = val - prevHdgValSb;
        switchBox = true;
    }

    if (val != INT_MIN) {
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
            sendEvent(KEY_HEADING_BUG_SET, newVal);
            lastHdgVal = newVal;

            if (switchBox) {
                prevHdgValSb = val;
            }
            else {
                prevHdgVal = val;
            }
        }
        time(&lastHdgAdjust);
    }
    else if (lastHdgAdjust != 0) {
        if (now - lastHdgAdjust > 3) {
            hdgSetSel = 0;
            lastHdgAdjust = 0;
            if (lastHdgVal != -1) {
                sendEvent(KEY_HEADING_BUG_SET, lastHdgVal);
            }
        }
    }

    // Heading push
    val = globals.gpioCtrl->readPush(headingControl);
    int prevVal = prevHdgPush;
    switchBox = false;

    if (ignore || prevHdgPushSb == 0) {
        prevHdgPushSb = simVars->sbButton[3];
    }
    else if (simVars->sbButton[3] != prevHdgPushSb) {
        val = simVars->sbButton[3];
        prevVal = prevHdgPushSb;
        switchBox = true;
    }

    if (val != INT_MIN) {
        if (prevVal % 2 == 1) {
            // Short press switches between 5 degree and 1 degree increments
            // Default is 1 degree
            if (hdgSetSel == 0) {
                hdgSetSel = 1;
            }
            else {
                hdgSetSel = 0;
            }
            time(&lastHdgAdjust);
            time(&lastHdgPush);
        }
        if (val % 2 == 1) {
            // Released
            lastHdgPush = 0;
        }
        if (switchBox) {
            prevHdgPushSb = val;
        }
        else {
            prevHdgPush = val;
        }
    }

    // Heading long push (over 1 sec)
    if (lastHdgPush > 0) {
        if (now - lastHdgPush > 1) {
            if (orbit > 0) {
                // Turn orbit off - Allow 20 degrees to stop turn
                if (orbit == 1) {
                    heading = simVars->hiHeading - 20.0;
                    if (heading < 0.0) {
                        heading += 360.0;
                    }
                }
                else {
                    heading = simVars->hiHeading + 20.0;
                    if (heading >= 360.0) {
                        heading -= 360.0;
                    }
                }

                sendEvent(KEY_HEADING_BUG_SET, heading);
                time(&lastHdgAdjust);
                orbit = 0;
            }
            else {
                // Long press switches between managed and selected when flight
                // director is active or toggles heading hold when it isn't.
                double setHeading = simVars->hiHeading;
                if (airliner && fdEnabled) {
                    manSelHeading();
                    if (!managedHeading) {
                        // Keep same heading when managed mode turned off
                        sendEvent(KEY_HEADING_BUG_SET, setHeading);
                    }
                }
                else if (autopilotHdg == HdgSet) {
                    if (apEnabled && !airliner) {
                        // Turn orbit on if heading bug is at least 90 degrees left or right of current heading
                        int diff = simVars->hiHeading - simVars->autopilotHeading;
                        if (diff < 0) {
                            diff += 360;
                        }

                        if (diff > 90 && diff < 180) {
                            orbit = 1;
                        }
                        else if (diff > 180 && diff < 270) {
                            orbit = 2;
                        }
                    }

                    if (orbit > 0) {
                        continueOrbit();
                    }
                    else {
                        autopilotHdg = LevelFlight;
                        sendEvent(KEY_AP_HDG_HOLD_OFF);
                        manSelHeading();
                        // Keep same heading when heading hold turned off
                        sendEvent(KEY_HEADING_BUG_SET, setHeading);
                    }
                }
                else {
                    autopilotHdg = HdgSet;
                    sendEvent(KEY_AP_HDG_HOLD_ON);
                    manSelHeading();
                }
            }
            hdgSetSel = 0;
            lastHdgPush = 0;
            time(&lastHdgAdjust);
        }
    }
}

void autopilot::gpioAltitudeInput()
{
    // Altitude rotate
    int val = globals.gpioCtrl->readRotation(altitudeControl);
    int diff = (val - prevAltVal) / 4;
    bool switchBox = false;

    if (simVars->sbMode > 1) {
        prevAltValSb = simVars->sbEncoder[1];
    }
    else if (simVars->sbEncoder[1] != prevAltValSb) {
        val = simVars->sbEncoder[1];
        diff = val - prevAltValSb;
        switchBox = true;
    }

    if (val != INT_MIN) {
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
            newAltitude(newVal);
            if (setVerticalSpeed != 0) {
                setAltitude = newVal;
            }
            if (switchBox) {
                prevAltValSb = val;
            }
            else {
                prevAltVal = val;
            }
        }
        time(&lastAltAdjust);
    }
    else if (lastAltAdjust != 0) {
        // Reset digit set selection if more than 3 seconds since last adjustment
        if (now - lastAltAdjust > 3) {
            altSetSel = 0;
            lastAltAdjust = 0;
        }
    }

    // Altitude push
    val = globals.gpioCtrl->readPush(altitudeControl);
    int prevVal = prevAltPush;
    switchBox = false;

    if (simVars->sbMode > 1 || prevAltPushSb == 0) {
        prevAltPushSb = simVars->sbButton[1];
    }
    else if (simVars->sbButton[1] != prevAltPushSb) {
        val = simVars->sbButton[1];
        prevVal = prevAltPushSb;
        switchBox = true;
    }

    if (val != INT_MIN) {
        if (prevVal % 2 == 1) {
            // Short press switches between 1000ft and 100ft increments
            if (altSetSel == 1) {
                altSetSel = 0;
            }
            else {
                altSetSel++;
            }
            time(&lastAltAdjust);
            time(&lastAltPush);
        }
        if (val % 2 == 1) {
            // Released
            lastAltPush = 0;
        }
        if (switchBox) {
            prevAltPushSb = val;
        }
        else {
            prevAltPush = val;
        }
    }

    // Altitude long push (over 1 sec)
    if (lastAltPush > 0) {
        if (now - lastAltPush > 1) {
            // Long press switches between managed and selected
            if (autopilotAlt == AltHold) {
                autopilotAlt = PitchHold;
                sendEvent(KEY_AP_ALT_HOLD_OFF);
            }
            else {
                autopilotAlt = AltHold;
                sendEvent(KEY_AP_ALT_HOLD_ON);
            }
            manSelAltitude();
            setVerticalSpeed = 0;
            altSetSel = 0;
            lastAltPush = 0;
            time(&lastAltAdjust);
        }
    }
}

void autopilot::gpioVerticalSpeedInput()
{
    // Vertical speed rotate
    int val = globals.gpioCtrl->readRotation(verticalSpeedControl);
    int diff = (val - prevVsVal) / 4;
    bool switchBox = false;

    if (simVars->sbMode > 1) {
        prevVsValSb = simVars->sbEncoder[0];
    }
    else if (simVars->sbEncoder[0] != prevVsValSb) {
        val = simVars->sbEncoder[0];
        diff = val - prevVsValSb;
        switchBox = true;
    }

    if (val != INT_MIN) {
        int adjust = 0;
        if (diff > 0) {
            adjust = 1;
        }
        else if (diff < 0) {
            adjust = -1;
        }

        if (adjust != 0) {
            if (simVars->autopilotVerticalHold == -1) {
                // Adust FPA
                double newVal = adjustFpa(adjust);
                // Expects FPA x 10
                newVerticalSpeed(newVal);
            }
            else {
                // Adjust vertical speed
                double newVal = adjustVerticalSpeed(adjust);
                newVerticalSpeed(newVal);
                lastVsVal = newVal;
                if (setVerticalSpeed != 0) {
                    setVerticalSpeed = newVal;
                }
            }
            if (switchBox) {
                prevVsValSb = val;
            }
            else {
                prevVsVal = val;
            }
        }
        time(&lastVsAdjust);
    }
    else if (lastVsAdjust != 0) {
        if (now - lastVsAdjust > 3) {
            lastVsAdjust = 0;
        }
    }

    // Vertical speed push
    val = globals.gpioCtrl->readPush(verticalSpeedControl);
    int prevVal = prevVsPush;
    switchBox = false;

    if (simVars->sbMode > 1 || prevVsPushSb == 0) {
        prevVsPushSb = simVars->sbButton[0];
    }
    else if (simVars->sbButton[0] != prevVsPushSb) {
        val = simVars->sbButton[0];
        prevVal = prevVsPushSb;
        switchBox = true;
    }

    if (val != INT_MIN) {
        // If in selected mode, short press switches between HDG,V/S and TRK,FPA mode.
        // If in managed mode, short press switches to selected mode.
        if (prevVal % 2 == 1) {
            if (simVars->autopilotVerticalHold == 0 || loadedAircraft != FBW_A320) {
                selectedVs();
            }
            else {
                // Switch between HDG,V/S and TRK,FPA mode
                sendEvent(A32NX_FCU_TRK_FPA_TOGGLE_PUSH);
            }
            time(&lastVsPush);
        }
        if (val % 2 == 1) {
            // Released
            lastVsPush = 0;
        }
        if (switchBox) {
            prevVsPushSb = val;
        }
        else {
            prevVsPush = val;
        }
    }

    // V/S long push (over 1 sec)
    if (lastVsPush > 0) {
        if (now - lastVsPush > 1) {
            // Long press switches to selected mode
            selectedVs();
            lastVsPush = 0;
        }
    }
}

void autopilot::gpioButtonsInput()
{
    // Autopilot push
    int val = globals.gpioCtrl->readPush(autopilotControl);
    int prevVal = prevApPush;
    int switchBox = false;

    if (simVars->sbMode > 1 || prevApPushSb == 0) {
        prevApPushSb = simVars->sbButton[6];
    }
    else if (simVars->sbButton[6] != prevApPushSb) {
        val = simVars->sbButton[6];
        prevVal = prevApPushSb;
        switchBox = true;
    }

    if (val != INT_MIN) {
        if (prevVal % 2 == 1) {
            // Toggle autopilot
            apEnabled = !apEnabled;
            globals.gpioCtrl->writeLed(autopilotControl, apEnabled);

            // Capture values if autopilot has been engaged
            if (apEnabled && loadedAircraft != AIRBUS_A310) {
                if (airliner && fdEnabled) {
                    captureInitial();
                }
                else {
                    captureCurrent();
                }
            }
            double value = 0;
            if (apEnabled) {
                value = 1;
            }
            sendEvent(KEY_AP_MASTER, value);

            if (apEnabled) {
                if (loadedAircraft == AIRBUS_A310 || loadedAircraft == CESSNA_LONGITUDE) {
                    // Enable autothrottle and flight director
                    athrEnabled = true;
                    globals.gpioCtrl->writeLed(autothrottleControl, athrEnabled);
                    sendEvent(KEY_AUTO_THROTTLE_ARM, 1);
                    fdEnabled = true;
                    globals.gpioCtrl->writeLed(flightDirectorControl, fdEnabled);
                    toggleFlightDirector();
                }
            }

            if (apEnabled && airliner && fdEnabled) {
                newAltitude(setAltitude);
            }
        }
        if (switchBox) {
            prevApPushSb = val;
        }
        else {
            prevApPush = val;
        }
        time(&lastApAdjust);
    }
    else if (lastApAdjust != 0) {
        if (now - lastApAdjust > 2) {
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
        if (now - lastFdAdjust > 2) {
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
            double value = 0;
            if (athrEnabled) {
                value = 1;
            }
            sendEvent(KEY_AUTO_THROTTLE_ARM, value);

            if (athrEnabled && loadedAircraft == AIRBUS_A310) {
                managedSpeed = false;
                sendEvent(KEY_AP_SPEED_SLOT_INDEX_SET, 1);
                sendEvent(KEY_AP_SPD_VAR_SET, 210);
            }
        }
        prevAthrPush = val;
        time(&lastAthrAdjust);
    }
    else if (lastAthrAdjust != 0) {
        if (now - lastAthrAdjust > 2) {
            lastAthrAdjust = 0;
        }
    }

    // Localiser push
    val = globals.gpioCtrl->readPush(localiserControl);
    prevVal = prevLocPush;
    switchBox = false;

    if (simVars->sbMode > 1 || prevLocPushSb == 0) {
        prevLocPushSb = simVars->sbButton[5];
    }
    else if (simVars->sbButton[5] != prevLocPushSb) {
        val = simVars->sbButton[5];
        prevVal = prevLocPushSb;
        switchBox = true;
    }

    if (val != INT_MIN) {
        if (prevVal % 2 == 1) {
            // Toggle localiser
            locEnabled = !locEnabled;
            globals.gpioCtrl->writeLed(localiserControl, locEnabled);
            if (apprEnabled) {
                sendEvent(KEY_AP_APR_HOLD_OFF);
            }
            double value = 0;
            if (locEnabled) {
                value = 1;
            }
            sendEvent(KEY_AP_LOC_HOLD, value);
        }
        if (switchBox) {
            prevLocPushSb = val;
        }
        else {
            prevLocPush = val;
        }
        time(&lastLocAdjust);
    }
    else if (lastLocAdjust != 0) {
        if (now - lastLocAdjust > 2) {
            lastLocAdjust = 0;
        }
    }

    // Approach push
    val = globals.gpioCtrl->readPush(approachControl);
    prevVal = prevApprPush;
    switchBox = false;

    if (simVars->sbMode > 1 || prevApprPushSb == 0) {
        prevApprPushSb = simVars->sbButton[4];
    }
    else if (simVars->sbButton[4] != prevApprPushSb) {
        val = simVars->sbButton[4];
        prevVal = prevApprPushSb;
        switchBox = true;
    }

    if (val != INT_MIN) {
        if (prevVal % 2 == 1) {
            // Toggle approach
            apprEnabled = !apprEnabled;
            globals.gpioCtrl->writeLed(approachControl, apprEnabled);

            if (apprEnabled) {
                sendEvent(KEY_AP_APR_HOLD_ON);
            }
            else {
                sendEvent(KEY_AP_APR_HOLD_OFF);
            }
        }
        if (switchBox) {
            prevApprPushSb = val;
        }
        else {
            prevApprPush = val;
        }
        time(&lastApprAdjust);
    }
    else if (lastApprAdjust != 0) {
        if (now - lastApprAdjust > 2) {
            lastApprAdjust = 0;
        }
    }
}

/// <summary>
/// Switch speed display between knots and mach
/// </summary>
void autopilot::machSwap()
{
    if (showMach) {
        showMach = false;
        if (loadedAircraft == AIRBUS_A310) {
            mach = simVars->autopilotAirspeed;
            sendEvent(KEY_AP_MACH_OFF);
            double celsius = 15 - 0.0019812 * simVars->altAltitude;
            double soundKnots = (sqrt(492 * (340 + celsius * 3.9)) - 107) * 1.94384;
            speed = mach * soundKnots;
            sendEvent(KEY_AP_SPD_VAR_SET, speed);
        }
    }
    else {
        showMach = true;
        if (loadedAircraft == AIRBUS_A310) {
            speed = simVars->autopilotAirspeed;
            sendEvent(KEY_AP_MACH_ON);
            double celsius = 15 - 0.0019812 * simVars->altAltitude;
            double soundKnots = (sqrt(492 * (340 + celsius * 3.9)) - 107) * 1.94384;
            mach = speed / soundKnots;
            sendEvent(KEY_AP_SPD_VAR_SET, mach);
        }
    }

    if (loadedAircraft == FBW_A320) {
        sendEvent(A32NX_FCU_SPD_MACH_TOGGLE_PUSH);
    }
}

/// <summary>
/// If flight director is enabled/disabled just after
/// take off initialise autopilot settings.
/// </summary>
void autopilot::toggleFlightDirector()
{
    if (!airliner) {
        return;
    }

    double value = 0;
    if (fdEnabled) {
        value = 1;
    }
    sendEvent(KEY_TOGGLE_FLIGHT_DIRECTOR, value);

    // Adjust autopilot settings if just after take off
    if (simVars->altAltitude > 4000 || simVars->vsiVerticalSpeed < 1) {
        return;
    }

    // Initial settings
    int holdSpeed = 210;
    setAltitude = 4000;
    setVerticalSpeed = 1500;

    if (fdEnabled) {
        // Use managed heading if FD turned on
        managedHeading = true;
        sendEvent(KEY_AP_HEADING_SLOT_INDEX_SET, 2);
    }
    else {
        // Use current heading if FD turned off
        managedHeading = false;
        sendEvent(KEY_AP_HEADING_SLOT_INDEX_SET, 1);
        sendEvent(KEY_HEADING_BUG_SET, simVars->hiHeading);
    }

    sendEvent(KEY_AP_ALT_HOLD_ON);
    if (simVars->autopilotAltitude < setAltitude) {
        newAltitude(setAltitude);
    }
    sendEvent(KEY_AP_AIRSPEED_ON);

    managedSpeed = false;
    sendEvent(KEY_AP_SPEED_SLOT_INDEX_SET, 1);
    if (simVars->autopilotAirspeed < holdSpeed) {
        sendEvent(KEY_AP_SPD_VAR_SET, holdSpeed);
    }

    managedAltitude = true;
    sendEvent(KEY_AP_VS_SLOT_INDEX_SET, 2);
    newVerticalSpeed(setVerticalSpeed);
}

/// <summary>
/// Autopilot engaged on airliner so capture values
/// that have already been set (or set minimums).
/// </summary>
void autopilot::captureInitial()
{
    // Initial settings
    int holdSpeed = simVars->autopilotAirspeed;
    if (holdSpeed < 210) {
        holdSpeed = 210;
    }

    setAltitude = simVars->autopilotAltitude;
    if (setAltitude < 3000) {
        setAltitude = 3000;
    }

    setVerticalSpeed = simVars->autopilotVerticalSpeed;
    if (setVerticalSpeed < 1500) {
        setVerticalSpeed = 1500;
    }

    // Use managed heading
    managedHeading = true;
    sendEvent(KEY_AP_HEADING_SLOT_INDEX_SET, 2);
    sendEvent(KEY_AP_PANEL_HEADING_SET, 0);

    sendEvent(KEY_AP_ALT_HOLD_ON);
    newAltitude(setAltitude);
    sendEvent(KEY_AP_AIRSPEED_ON);

    managedSpeed = false;
    sendEvent(KEY_AP_SPEED_SLOT_INDEX_SET, 1);
    sendEvent(KEY_AP_SPD_VAR_SET, holdSpeed);

    managedAltitude = true;
    sendEvent(KEY_AP_VS_SLOT_INDEX_SET, 2);
    newVerticalSpeed(setVerticalSpeed);
}

/// <summary>
/// Switch between managed and selected speed.
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
        sendEvent(KEY_AP_SPEED_SLOT_INDEX_SET, 2);
    }
    else {
        sendEvent(KEY_AP_SPEED_SLOT_INDEX_SET, 1);
    }
}

/// <summary>
/// Switch between managed and selected heading.
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
        sendEvent(KEY_AP_HEADING_SLOT_INDEX_SET, 2);
        sendEvent(KEY_AP_PANEL_HEADING_SET, 0);
    }
    else {
        sendEvent(KEY_AP_HEADING_SLOT_INDEX_SET, 1);
        sendEvent(KEY_AP_PANEL_HEADING_SET, 1);
    }
}

/// <summary>
/// Switch between managed and selected altitude.
/// </summary>
void autopilot::manSelAltitude()
{
    if (!airliner) {
        return;
    }

    managedAltitude = !managedAltitude;

    if (managedAltitude) {
        sendEvent(KEY_AP_VS_SLOT_INDEX_SET, 2);
    }
    else {
        sendEvent(KEY_AP_VS_SLOT_INDEX_SET, 1);
    }
}

void autopilot::selectedVs()
{
    autopilotAlt = VerticalSpeedHold;
    if (loadedAircraft == AIRBUS_A310) {
        sendEvent(KEY_AP_VS_SET);
    }
    newAltitude(simVars->autopilotAltitude);

    sendEvent(KEY_AP_ALT_HOLD_ON);

    if (loadedAircraft == BOEING_747) {
        // B747 Bug - Try to force aircraft into VS mode
        manSelAltitude();
    }
    captureVerticalSpeed();
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

    sendEvent(KEY_AP_SPD_VAR_SET, holdSpeed);

    if (!managedHeading) {
        sendEvent(KEY_HEADING_BUG_SET, simVars->hiHeading);
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
        newAltitude(holdAlt);
    }
}

void autopilot::captureVerticalSpeed()
{
    setAltitude = simVars->autopilotAltitude;

    if (setAltitude < simVars->altAltitude && simVars->autopilotVerticalSpeed >= 0) {
        setVerticalSpeed = -700;
        sendEvent(KEY_AP_VS_SET, 1);
        newVerticalSpeed(setVerticalSpeed);
    }
    else if (setAltitude > simVars->altAltitude && simVars->autopilotVerticalSpeed <= 0) {
        setVerticalSpeed = 700;
        sendEvent(KEY_AP_VS_SET, 1);
        newVerticalSpeed(setVerticalSpeed);
    }
    else {
        setVerticalSpeed = simVars->autopilotVerticalSpeed;
    }
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
        // Adjust fives
        heading += adjust * 5;
    }
    else {
        heading += adjust;
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

    int diff = abs(altitude - simVars->altAltitude);
    if (diff > 400) {
        if (altitude < simVars->altAltitude && setVerticalSpeed >= 0) {
            setVerticalSpeed = -700;
            sendEvent(KEY_AP_VS_SET, 1);
            newVerticalSpeed(setVerticalSpeed);
        }
        else if (altitude > simVars->altAltitude && setVerticalSpeed <= 0) {
            setVerticalSpeed = 700;
            sendEvent(KEY_AP_VS_SET, 1);
            newVerticalSpeed(setVerticalSpeed);
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

double autopilot::adjustFpa(int adjust)
{
    // Adjust in 0.1 degree increments between -9.9 and 9.9
    fpaX10 += adjust;

    if (fpaX10 < -99) {
        fpaX10 = -99;
    }
    else if (fpaX10 > 99)
    {
        fpaX10 = 99;
    }

    return fpaX10;
}

/// <summary>
/// Hold mode for GA aircraft with autopilot.
/// Keeps setting heading bug 90 degrees left or right of
/// current track so aircraft will continue to orbit.
/// </summary>
void autopilot::continueOrbit()
{
    if (!apEnabled) {
        orbit = 0;
        return;
    }

    if (orbitDelay > 0) {
        orbitDelay--;
    }
    else {
        orbitDelay = 5;

        if (orbit == 1) {
            heading = simVars->hiHeading - 90.0;
            if (heading < 0.0) {
                heading += 360.0;
            }
        }
        else {
            heading = simVars->hiHeading + 90.0;
            if (heading >= 360.0) {
                heading -= 360.0;
            }
        }

        sendEvent(KEY_HEADING_BUG_SET, heading);
        time(&lastHdgAdjust);
    }
}

void autopilot::newAltitude(double newVal)
{
    sendEvent(KEY_AP_ALT_VAR_SET_ENGLISH, newVal);
    lastAltVal = newVal;
    altSetRetry = 30;
}

void autopilot::newVerticalSpeed(double newVal)
{
    sendEvent(KEY_AP_VS_VAR_SET_ENGLISH, newVal);
    lastVsVal = newVal;
    vsSetRetry = 30;
}
