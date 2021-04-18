#ifndef _SIMVARS_H_
#define _SIMVARS_H_

#include <stdio.h>
#include <unistd.h>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
typedef int SOCKET;
#define SOCKET_ERROR -1
#define INVALID_SOCKET -1
#define SOCKADDR sockaddr
#define closesocket close
#include "globals.h"
#include "simvarDefs.h"

extern globalVars globals;

class simvars {
public:
    SimVars simVars;

private:
    std::thread* dataLinkThread = NULL;

    SOCKET writeSockfd = INVALID_SOCKET;
    sockaddr_in writeAddr;
    Request writeRequest;

public:
    simvars();
    ~simvars();
    void write(EVENT_ID eventId, double value = 0);
};

#endif // _SIMVARS_H_
