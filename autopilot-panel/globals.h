#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <cstring>
#define _stricmp strcasecmp

class settings;
class simvars;
class gpioctrl;

enum Aircraft {
    UNDEFINED,
    NO_AIRCRAFT,
    CESSNA_152,
    CESSNA_172,
    CESSNA_CJ4,
    SAVAGE_CUB,
    SHOCK_ULTRA,
    FBW_A320,
    BOEING_747,
    SUPERMARINE_SPITFIRE,
    OTHER_AIRCRAFT,
    OTHER_AIRCRAFT2,
};

struct globalVars
{
    const char* Cessna_152_Text = "Cessna 152";
    const int Cessna_152_Len = 10;
    const char* Cessna_172_Text = "Cessna Skyhawk";
    const int Cessna_172_Len = 14;
    const char* Cessna_CJ4_Text = "Cessna CJ4";
    const int Cessna_CJ4_Len = 10;
    const char* Savage_Cub_Text = "Asobo Savage Cub";
    const int Savage_Cub_Len = 16;
    const char* Shock_Ultra_Text = "Savage Shock Ultra";
    const int Shock_Ultra_Len = 18;
    const char* Airbus_A320_Text = "Airbus A320";
    const int Airbus_A320_Len = 11;
    const char* FBW_A320_Text = "FBW";
    const int FBW_A320_Len = 3;
    const char* Boeing_747_Text = "Boeing 747";
    const int Boeing_747_Len = 10;
    const char* Supermarine_Spitfire_Text = "FlyingIron Spitfire";
    const int Supermarine_Spitfire_Len = 19;
    const int FastAircraftSpeed = 195;

    const char* BitmapDir = "bitmaps/";
    const char* SettingsDir = "settings/";
    const char* SettingsFile = "settings/autopilot-panel.json";

    settings* allSettings = NULL;
    simvars* simVars = NULL;
    gpioctrl* gpioCtrl = NULL;
    Aircraft aircraft;
    char lastAircraft[32];

    long dataRateFps = 16;
    bool quit = false;
    bool dataLinked = false;
    bool connected = false;
    bool electrics = false;
    bool avionics = false;
};

#endif // _GLOBALS_H_
