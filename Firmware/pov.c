#include "GenericTypeDefs.h"
#include "Compiler.h"

#include "pov.h"
#include "font.h"
#include "HardwareProfile - Low Pin Count USB Development Kit.h"

extern char message[];
extern unsigned int space, POV_curPos, POV_curLetter;

void testLEDs()
{
    int i; unsigned int t;

    for(i = 0; i < 3; i++)
    {
        setLEDs(0xAA);
        for(t=0;t<65535;t++);
        setLEDs(0x55);
        for(t=0;t<65535;t++);
    }

	setLEDs(0x00);
}

void setLEDs(unsigned char pattern)
{
    LED1 = pattern & 1;
    LED2 = (pattern & (1 << 1)) >> 1;
    LED3 = (pattern & (1 << 2)) >> 2;
    LED4 = (pattern & (1 << 3)) >> 3;
    LED5 = (pattern & (1 << 4)) >> 4;
    LED6 = (pattern & (1 << 5)) >> 5;
    LED7 = (pattern & (1 << 6)) >> 6;
}

void POVRoutine(void)
{
	unsigned char i;
	unsigned char rom *ptr = font;

	// Set LEDs to pattern
	if (space == 0)
	{
		// Increment pointer position
        for(i=0;i<5;i++)
            ptr += message[POV_curLetter]-0x20;
        ptr += POV_curPos;

		setLEDs(*ptr);
		POV_curPos++;
	}
	else
	{
		space--;
		setLEDs(0);
	}

	if (POV_curPos == 5)
	{
		POV_curPos = 0;
		POV_curLetter++;
		space = 2;
		if (POV_curLetter == strlen(message))
		{
			POV_curLetter = 0;
			space=20;
		}
	}
}