#ifndef _SEVENSEGMENT_H_
#define _SEVENSEGMENT_H_

class sevensegment
{
private:
	int channel;
	unsigned char prevDisplay[3][8];	// Max 3 displays daisy chained

public:
	sevensegment(bool initWiringPi, int spiChannel);
	void getSegData(unsigned char* buf, int bufSize, int num, int fixedSize);
	void blankSegData(unsigned char* buf, int bufSize, bool wantMinus);
	void decimalSegData(unsigned char* buf, int pos);
	void writeSegData3(unsigned char* buf1, unsigned char* buf2, unsigned char* buf3);

private:
	void writeSegHex(int display, char* hex);
	void writeSegData(int display, unsigned char* buf);
};

#endif // _SEVENSEGMENT_H_
