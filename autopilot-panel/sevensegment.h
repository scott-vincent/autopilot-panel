#ifndef _SEVENSEGMENT_H_
#define _SEVENSEGMENT_H_

class sevensegment
{
public:
	sevensegment(int channel);
	void getSegData(unsigned char* buf, int bufSize, int num, int fixedSize);
	void blankSegData(unsigned char* buf, int bufSize, bool showMinus);
	void decimalSegData(unsigned char* buf, int pos);
	void writeSegData3(unsigned char* buf1, unsigned char* buf2, unsigned char* buf3);

private:
	void writeSegHex(int display, char* hex);
};

#endif // _SEVENSEGMENT_H_
