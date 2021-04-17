echo Building autopilot-panel
cd autopilot-panel
g++ -lwiringPi -lpthread  \
    -o autopilot-panel \
    -I . \
    settings.cpp \
    simvarDefs.cpp \
    simvars.cpp \
    globals.cpp \
    gpioctrl.cpp \
    sevensegment.cpp \
    autopilot.cpp \
    autopilot-panel.cpp \
    || exit
echo Done
