#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "settings.h"
#include "simvars.h"

const char *DataLinkGroup = "Data Link";
char dataLinkHost[64];
int dataLinkPort;

extern const char* SimVarDefs[][2];
char deltaData[2048];

void dataLink(simvars*);
void identifyAircraft(char* aircraft);
void receiveDelta(char* deltaData, long deltaSize, char* simVarsPtr);

simvars::simvars()
{
    globals.allSettings->getString(DataLinkGroup, "Host", dataLinkHost);
    if (dataLinkHost == NULL) {
        strcpy(dataLinkHost, "127.0.0.1");
    }

    dataLinkPort = globals.allSettings->getInt(DataLinkGroup, "Port");
    if (dataLinkPort == INT_MIN) {
        dataLinkPort = 52020;
    }

    // Start data link thread
    dataLinkThread = new std::thread(dataLink, this);
}

simvars::~simvars()
{
    if (dataLinkThread) {
        // Wait for thread to exit
        dataLinkThread->join();
    }
}

/// <summary>
/// Write event to Flight Sim with optional data value
/// </summary>
void simvars::write(EVENT_ID eventId, double value)
{
    if (!globals.dataLinked) {
        return;
    }

    request.requestedSize = sizeof(WriteData);
    request.writeData.eventId = eventId;
    request.writeData.value = value;

    if (writeSockfd == INVALID_SOCKET) {
        if ((writeSockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET) {
            printf("Failed to create UDP socket for writing\n");
            fflush(stdout);
            return;
        }

        int opt = 1;
        setsockopt(writeSockfd, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));

        writeAddr.sin_family = AF_INET;
        writeAddr.sin_port = htons(dataLinkPort);
        inet_pton(AF_INET, dataLinkHost, &writeAddr.sin_addr);
    }

    int bytes = sendto(writeSockfd, (char*)&request, sizeof(request), 0, (SOCKADDR*)&writeAddr, sizeof(writeAddr));
    if (bytes <= 0) {
        printf("Failed to write event %d\n", eventId);
        fflush(stdout);
    }
}

/// <summary>
/// A separate thread constantly collects the latest
/// SimVar values from instrument-data-link.
/// </summary>
void dataLink(simvars* t)
{
    // Create a UDP socket
    SOCKET sockfd;
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET) {
        printf("DataLink: Failed to create UDP socket\n");
        exit(1);
    }

    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(dataLinkPort);
    if (inet_pton(AF_INET, dataLinkHost, &addr.sin_addr) <= 0)
    {
        printf("DataLink: Invalid server address: %s\n", dataLinkHost);
        exit(1);
    }

    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 500000;

    // Only want a subset of SimVars for Autopilot panel (to save bandwidth)
    long dataSize = (long)(&t->simVars.autothrottleActive) + sizeof(double) - (long)&t->simVars;
    Request request;
    request.requestedSize = dataSize;
    request.wantFullData = 1;   // Want full data on first connect
    long actualSize;
    int bytes;

    // Detect current aircraft and convert to an int.
    // We do this here to save having to do it for each instrument.
    globals.aircraft = NO_AIRCRAFT;
    strcpy(globals.lastAircraft, "");

    printf("Waiting for Data Link at %s:%d\n", dataLinkHost, dataLinkPort);
    fflush(stdout);
    bool prevConnected = false;
    int selFail = 0;

    while (!globals.quit) {
        // Poll instrument data link
        bytes = sendto(sockfd, (char*)&request, sizeof(request), 0, (SOCKADDR*)&addr, sizeof(addr));

        if (bytes > 0) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(sockfd, &fds);

            int sel = select(FD_SETSIZE, &fds, 0, 0, &timeout);
            if (sel > 0) {
                selFail = 0;
                // Receive latest data (delta will never be larger than full data size)
                bytes = recv(sockfd, deltaData, dataSize, 0);

                if (bytes == dataSize) {
                    // Full data received
                    memcpy((char*)&t->simVars, deltaData, dataSize);
                    request.wantFullData = 0;   // Want deltas from now on
                    globals.connected = (t->simVars.connected == 1);

                    if (!globals.dataLinked) {
                        globals.dataLinked = true;
                        printf("Established Data Link at %s:%d\n", dataLinkHost, dataLinkPort);
                        if (!globals.connected) {
                            printf("Waiting for MS FS2020\n");
                        }
                        fflush(stdout);
                    }

                    if (globals.connected != prevConnected) {
                        if (globals.connected) {
                            printf("Connected to MS FS2020\n");
                        }
                        else {
                            printf("Waiting for MS FS2020\n");
                        }
                        fflush(stdout);
                        prevConnected = globals.connected;
                    }

                    identifyAircraft(t->simVars.aircraft);
                }
                else if (bytes == sizeof(long)) {
                    // Data size mismatch
                    memcpy(&actualSize, &t->simVars, sizeof(long));
                    printf("DataLink: Requested %ld bytes but server sent %ld bytes\n", dataSize, actualSize);
                    fflush(stdout);
                    exit(1);
                }
                else if (bytes > 0) {
                    // Delta received
                    receiveDelta(deltaData, bytes, (char*)&t->simVars);
                    identifyAircraft(t->simVars.aircraft);
                }
                else {
                    bytes = SOCKET_ERROR;
                }
            }
            else {
                // Link can blip so wait for multiple failures
                selFail++;
                if (selFail > 15) {
                    bytes = SOCKET_ERROR;
                }
            }
        }
        else {
            bytes = SOCKET_ERROR;
        }

        if (bytes == SOCKET_ERROR && globals.dataLinked) {
            globals.dataLinked = false;
            globals.connected = false;
            globals.aircraft = NO_AIRCRAFT;
            strcpy(globals.lastAircraft, "");
            request.wantFullData = 1;
            printf("Waiting for Data Link at %s:%d\n", dataLinkHost, dataLinkPort);
            fflush(stdout);
        }

        // Update 16 times per second
        usleep(62500);
    }

    closesocket(sockfd);
}
