#include "ReelTwo.h"

ConsoleSerial Serial(stdin, stdout);

void setup();
void loop();

int main(int argc, const char* argv[])
{
	setup();
	for (;;)
	{
		loop();
	}
	return 0;
}