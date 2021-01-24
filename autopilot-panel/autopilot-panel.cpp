/*
 * Flight Simulator Autopilot Panel
 * Copyright (c) 2021 Scott Vincent
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "gpioctrl.h"
#include "globals.h"
#include "settings.h"
#include "simvars.h"
#include "autopilot.h"

const char* autopilotVersion = "v1.0.0";
const bool Debug = false;

struct globalVars globals;

autopilot* ap;

/// <summary>
/// Initialise
/// </summary>
void init(const char *settingsFile = NULL)
{
    globals.allSettings = new settings(settingsFile);
    globals.simVars = new simvars();
    globals.gpioCtrl = new gpioctrl();
}

/// <summary>
/// Fetch latest values of common variables
/// </summary>
void updateCommon()
{
    SimVars* simVars = &globals.simVars->simVars;

    // Electrics check
    globals.electrics = (simVars->dcVolts > 0);

    // Avionics check
    globals.avionics = (simVars->com1Status == 0 || simVars->com2Status == 0);
}

/// <summary>
/// Update everything before the next frame
/// </summary>
void doUpdate()
{
    updateCommon();

    ap->update();
}

///
/// main
///
int main(int argc, char **argv)
{
    printf("autopilot-panel %s\n", autopilotVersion);
    fflush(stdout);

    if (argc > 1) {
        init(argv[1]);
    }
    else {
        init();
    }

    ap = new autopilot();

    while (!globals.quit) {
        doUpdate();
        ap->render();

        // Update 10 times per second
        usleep(100000);
    }

    return 0;
}
