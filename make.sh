echo Building autopilot-panel
cd autopilot-panel
g++ -o autopilot-panel -I . \
    settings.cpp \
    simvarDefs.cpp \
    simvars.cpp \
    globals.cpp \
    gpioctrl.cpp \
    sevensegment.cpp \
    autopilot.cpp \
    autopilot-panel.cpp \
    -lwiringPi -lpthread || exit
echo Done
