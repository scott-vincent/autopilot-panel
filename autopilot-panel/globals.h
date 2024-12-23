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
    CESSNA_LONGITUDE,
    SAVAGE_CUB,
    SHOCK_ULTRA,
    AIRBUS_A310,
    FBW,
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
    const char* Cessna_Longitude_Text = "Cessna Longitude";
    const int Cessna_Longitude_Len = 16;
    const char* Savage_Cub_Text = "Asobo Savage Cub";
    const int Savage_Cub_Len = 16;
    const char* Shock_Ultra_Text = "Savage Shock Ultra";
    const int Shock_Ultra_Len = 18;
    const char* Boeing_747_Text = "Boeing 747-8";
    const int Boeing_747_Len = 12;
    const char* Salty_Boeing_747_Text = "Salty Boeing 747";
    const int Salty_Boeing_747_Len = 16;
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
