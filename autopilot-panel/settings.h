#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#include <climits>
#include "globals.h"

extern globalVars globals;

struct setting {
    char group[256];
    char name[256];
    char value[256];
};

class settings
{
private:
    int settingCount = 0;
    setting allSettings[256];

public:
    settings(const char* settingsFile);
    void getString(const char* group, const char* name, char* str);
    int getInt(const char* group, const char* name);

private:
    void readString(char* buf, int* pos, char* name);
    void readValue(char* buf, int* pos, char* value);
    void add(const char* group, const char* name, char* value);
};

#endif // _SETTINGS_H_
