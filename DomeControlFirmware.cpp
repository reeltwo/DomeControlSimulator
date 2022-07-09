#define HIGH 1
#define LOW 0

bool digitalRead(unsigned pin)
{
	return false;
}
void digitalWrite(unsigned pin, bool val)
{
}
void pinMode(unsigned pin, unsigned char m)
{
}

#include "../DomeControlFirmware/DomeControlFirmware.ino"

ConsoleSerial Serial(stdin, stdout);

int main(int argc, const char* argv[])
{
	setup();
	for (;;)
	{
		loop();
	}
	return 0;
}