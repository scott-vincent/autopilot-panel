echo Building autopilot-panel
cd autopilot-panel
g++ -lwiringPi -lpthread  \
    -o autopilot-panel \
    -I . \
    settings.cpp \
    simvarDefs.cpp \
    simvars.cpp \
    gpioctrl.cpp \
    autopilot.cpp \
    autopilot-panel.cpp \
    || exit
echo Done
