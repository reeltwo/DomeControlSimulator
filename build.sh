#!/bin/sh
CDEFS="-g -DARDUINO=10810 -DARDUINO_ARCH_LINUX=1 -I. -Isim -Isim/linux -I../DomeControlFirmware -I ../libraries/Reeltwo/src"
mkdir -p obj
g++ -c $CDEFS sim/Print.cpp -o obj/Print.o && \
g++ -c $CDEFS sim/Stream.cpp -o obj/Stream.o && \
g++ -c $CDEFS sim/EEPROM.cpp -o obj/EEPROM.o && \
g++ -c $CDEFS sim/stdlib_noniso.cpp -o obj/stdlib_noniso.o && \
g++ -c $CDEFS sim/WString.cpp -o obj/WString.o && \
g++ $CDEFS DomeControlFirmware.cpp -o DomeControlTester obj/stdlib_noniso.o obj/Stream.o obj/WString.o obj/Print.o obj/EEPROM.o -lm
