#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <wiringPiSPI.h>
#include "sevensegment.h"

///
/// This class allows you to drive a daisy-chained
/// set of 8 digit 7-segment displays using SPI.
/// 
/// Wiring
/// ------
/// VCC - 3.3v = Pin 1 (does not work correctly if 5v used!)
/// GND - Ground = Pin 6
/// DIN - GPIO 10 (MOSI) = Pin 19
/// CS - GPIO 8 (CE0) = Pin 24
/// CLK - GPIO 11 (SCLK) = Pin 23
/// DOUT - Not connected unless daisy chaining (see below)
/// 
/// To daisy-chain connect DOUT from one module to DIN on
/// the next one. Connect all other pins in parallel.
///
/// Note: SPI must be enabled on the Pi so either use raspi-config
/// or edit /boot/config.txt and add "dtparm=spi=on" then reboot.
///

/// <summary>
/// Specify Channel 0 if using CE0 or channel 1 if using CE1.
/// </summary>
sevensegment::sevensegment(int channel)
{
    char hex[256];

    // Bus speed is 10 MHz
    wiringPiSPISetup(channel, 10000000);

    // Intialise all 3 displays
    strcpy(hex, "0f000c010a0109ff0b07010f020f030f040f050f060f070f080f");
    writeSegHex(1, hex);
    writeSegHex(2, hex);
    writeSegHex(3, hex);

    // Display 2 is very dim so make it max intensity!
    strcpy(hex, "0a0f");
    writeSegHex(2, hex);
}

/// <summary>
// Converts a number to segment display data.
// Leading zeroes are added up to fixedDigits size.
// Minus sign added if number negative and bufSize > fixedDigits.
/// </summary>
void sevensegment::getSegData(unsigned char* buf, int bufSize, int num, int fixedSize)
{
    bool minus = false;

    if (num < 0) {
        minus = true;
        num = -num;
    }

    // Work back from least significant digit
    int pos = bufSize - 1;
    for (; pos >= 0; pos--) {
        buf[pos] = num % 10;
        num /= 10;
        if (num == 0) {
            pos--;
            break;
        }
    }

    // Add leading zeroes if required
    if (pos >= 0 && fixedSize > 0) {
        int firstDigit = bufSize - fixedSize;
        for (; pos >= firstDigit; pos--) {
            // Pad with zero
            buf[pos] = 0;
        }
    }

    // If there is still room add minus sign and blanks
    if (pos >= 0) {
        if (minus) {
            buf[pos] = 0x0a;
            pos--;
        }

        for (; pos >= 0; pos--) {
            // Pad with blank
            buf[pos] = 0x0f;
        }
    }
}

/// <summary>
/// Blanks the display at the specified position.
/// If showMinus is true a minus is displayed at the rightmost position.
/// </summary>
void sevensegment::blankSegData(unsigned char* buf, int bufSize, bool showMinus)
{
    for (int pos = 0; pos < bufSize; pos++) {
        buf[pos] = 0x0f;
    }

    if (showMinus) {
        buf[bufSize-1] = 0x0a;
    }
}

/// <summary>
/// Adds a decimal point at the specified position.
/// </summary>
void sevensegment::decimalSegData(unsigned char* buf, int pos)
{
    buf[pos] = buf[pos] | 0x80;
}

/// <summary>
/// Takes 3 complete data buffers (8 chars each)
/// and writes them to 3 displays concurrently.
/// buf1 = left display, buf2 = middle, buf3 = right.
/// </summary>
void sevensegment::writeSegData3(unsigned char* buf1, unsigned char* buf2, unsigned char* buf3)
{
    unsigned char regData[6];

    // Write to pos 8 on 3 displays, pos 7 on 3 displays etc.
    int digitPos = 8;
    for (int i = 0; i < 8; i++) {
        regData[0] = digitPos;
        regData[1] = buf3[i];
        regData[2] = digitPos;
        regData[3] = buf2[i];
        regData[4] = digitPos;
        regData[5] = buf1[i];
        digitPos--;

        wiringPiSPIDataRW(0, regData, 6);
        usleep(500);
    }

    // Display 3 can get corrupted so send its data again!
    digitPos = 8;
    for (int i = 0; i < 8; i++) {
        regData[0] = digitPos;
        regData[1] = buf3[i];
        regData[2] = 0;  // No-op for display2
        regData[4] = 0;  // No-op for display1
        digitPos--;

        wiringPiSPIDataRW(0, regData, 6);
        usleep(500);
    }
}

/// <summary>
/// Write hex data to the specified display.
/// Note: display 1 is right-most display.
/// </summary>
void sevensegment::writeSegHex(int display, char* hex)
{
    unsigned char regData[6];
    int strLen = strlen(hex);
    char ch;
    int num;

    for (int i = 0; i < strLen; i += 4) {
        ch = hex[i + 4];
        hex[i + 4] = '\0';
        num = strtol(&hex[i], NULL, 16);
        hex[i + 4] = ch;

        int dataLen = 0;
        for (int j = 1; j <= 3; j++) {
            if (j == display) {
                regData[dataLen] = (num & 0xff00) >> 8;
                regData[dataLen + 1] = num & 0x00ff;
            }
            else {
                // No-op for this display
                regData[dataLen] = 0;
                regData[dataLen + 1] = 0;
            }

            dataLen += 2;
        }

        wiringPiSPIDataRW(0, regData, dataLen);
        usleep(500);
    }
}
