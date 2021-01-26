#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <wiringPi.h>
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
/// Note: SPI must be enabled on the Pi and we only want a
/// single channel with no MISO (to save pins) so you must
/// edit /boot/config.txt and add "dtoverlay=spi0-1cs,no_miso".
/// Don't use "dtparm=spi=on" as this uses up 2 extra pins.
///

/// <summary>
/// Specify Channel 0 if using CE0 or channel 1 if using CE1.
/// </summary>
sevensegment::sevensegment(bool initWiringPi, int spiChannel)
{
    char hex[256];

    channel = spiChannel;

    // Caller may want to initialise wiringPi themselves
    if (initWiringPi) {
        // Init wiring pi, needed for delayMicroseconds
        wiringPiSetupGpio();
    }

    // Init wiring pi SPI, get corruption at 10 MHz so use 1 MHz
    wiringPiSPISetup(channel, 1000000);

    // Intialise all 3 displays. Displays hyphens to show
    // displays have been initialised successfully.
    strcpy(hex, "0f000c010a0309ff0b07010a020a030a040a050a060a070a080a");
    writeSegHex(1, hex);
    writeSegHex(2, hex);
    writeSegHex(3, hex);

    // Clear displays after a short delay
    delayMicroseconds(1500000);
    strcpy(hex, "010f020f030f040f050f060f070f080f");
    writeSegHex(1, hex);
    writeSegHex(2, hex);
    writeSegHex(3, hex);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 8; j++) {
            prevDisplay[i][j] = 0x0f;
        }
    }
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
/// If wantMinus is true minus signs are displayed instead of blanks.
/// </summary>
void sevensegment::blankSegData(unsigned char* buf, int bufSize, bool wantMinus)
{
    for (int pos = 0; pos < bufSize; pos++) {
        if (wantMinus) {
            buf[pos] = 0x0a;
        }
        else {
            buf[pos] = 0x0f;
        }
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

        wiringPiSPIDataRW(channel, regData, dataLen);
        delayMicroseconds(500);
    }
}

/// <summary>
/// Takes 3 complete data buffers (8 chars each)
/// and writes them to 3 displays concurrently.
/// buf1 = left display, buf2 = middle, buf3 = right.
/// </summary>
void sevensegment::writeSegData3(unsigned char* buf1, unsigned char* buf2, unsigned char* buf3)
{
    // Display 0 is the last one (rightmost) in the chain
    writeSegData(2, buf1);
    writeSegData(1, buf2);
    writeSegData(0, buf3);
}

/// <summary>
/// Update a display but only write the digits that have changed.
/// Display must be 0 (right), 1 (middle) or 2 (left)
/// </summary>
void sevensegment::writeSegData(int display, unsigned char* buf)
{
    unsigned char regData[6];

    for (int i = 0; i < 8; i++) {
        if (buf[i] != prevDisplay[display][i]) {
            prevDisplay[display][i] = buf[i];

            // Set no-op for all displays
            regData[0] = 0;
            regData[2] = 0;
            regData[4] = 0;

            // Replace no-op with digit for required display
            // 1 = rightmost digit
            regData[display * 2] = 8 - i;
            regData[display * 2 + 1] = buf[i];

            // Send the data for single digit, single display
            wiringPiSPIDataRW(channel, regData, 6);
            delayMicroseconds(500);
        }
    }
}
