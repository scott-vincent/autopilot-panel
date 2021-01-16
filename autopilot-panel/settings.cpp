#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "settings.h"

settings::settings(const char* customSettings)
{
    char settingsFile[256];

    if (customSettings == NULL) {
        strcpy(settingsFile, globals.SettingsFile);
    }
    else if (strchr(customSettings, '/') == NULL && strchr(customSettings, '\\') == NULL) {
        sprintf(settingsFile, "%s%s", globals.SettingsDir, customSettings);
    }
    else {
        strcpy(settingsFile, customSettings);
    }

    printf("Reading %s\n", settingsFile);

    // Load settings from JSON file
    static char buf[16384] = { '\0' };

    // Read file in a single chunk
    FILE* infile = fopen(settingsFile, "r");
    if (infile) {
        int bytes = (int)fread(buf, 1, 16384, infile);
        buf[bytes] = '\0';
        fclose(infile);
    }
    else {
        printf("Settings file %s not found\n", settingsFile);
        exit(1);
    }

    char parent[8][256];
    int level = 0;

    int pos = 0;
    char ch;
    while ((ch = buf[pos]) != '\0') {
        if (ch == '{') {
            if (level > 6) {
                printf("Level too deep: %s\n", &buf[pos]);
                exit(1);
            }
            level++;
            pos++;
        }
        else if (ch == '}') {
            if (level == 0) {
                printf("Mismatched bracket: %s\n", &buf[pos]);
                exit(1);
            }
            level--;
            pos++;
        }
        else if (ch == '"') {
            // Read attribute name
            char name[256];
            readString(buf, &pos, name);

            if (buf[pos] != ':') {
                printf("missing colon: ");
                exit(1);
            }
            pos++;

            char value[256];
            readValue(buf, &pos, value);

            if (strcmp(value, "{") == 0) {
                strcpy(parent[level], name);
            }
            else {
                // Store new value
                char group[256];
                strcpy(group, parent[1]);

                for (int i = 2; i < level; i++) {
                    strcat(group, "/");
                    strcat(group, parent[i]);
                }

                add(group, name, value);
            }
        }
        else {
            // Ignore char
            pos++;
        }
    }
}

void settings::readString(char* buf, int* pos, char* str)
{
    char ch;

    (*pos)++;
    int nameStart = *pos;

    while ((ch = buf[*pos]) != '"') {
        if (ch == '\0') {
            printf("Missing end quote: %s\n", &buf[nameStart - 1]);
            exit(1);
        }
        (*pos)++;
    }

    int nameLen = *pos - nameStart;

    if (nameLen == 0 || nameLen > 64) {
        printf("Bad string: %s\n", &buf[nameStart - 1]);
        exit(1);
    }

    strncpy(str, &buf[nameStart], nameLen);
    str[nameLen] = '\0';
    (*pos)++;
}

void settings::readValue(char* buf, int* pos, char* value)
{
    char ch;

    while ((ch = buf[*pos]) == ' ') {
        (*pos)++;
    }

    if (ch == '\0' || ch == '\r' || ch == '\n') {
        printf("Line must end with a value or an opening curly brace");
        exit(1);
    }

    if (ch == '{') {
        strcpy(value, "{");
        return;
    }

    if (ch == '"') {
        readString(buf, pos, value);
        return;
    }

    int val = atoi(&buf[*pos]);
    sprintf(value, "%d", val);

    // No need to skip over value as main loop will do that
}

void settings::add(const char *group, const char *name, char *value)
{
    strcpy(allSettings[settingCount].group, group);
    strcpy(allSettings[settingCount].name, name);
    strcpy(allSettings[settingCount].value, value);
    settingCount++;
}

void settings::getString(const char* group, const char* name, char* str)
{
    for (int i = 0; i < settingCount; i++) {
        if (strcmp(allSettings[i].group, group) == 0 && strcmp(allSettings[i].name, name) == 0) {
            strcpy(str, allSettings[i].value);
            return;
        }
    }

    str = NULL;
}

int settings::getInt(const char* group, const char* name)
{
    for (int i = 0; i < settingCount; i++) {
        if (strcmp(allSettings[i].group, group) == 0 && strcmp(allSettings[i].name, name) == 0) {
            return atoi(allSettings[i].value);
        }
    }

    return INT_MIN;
}
