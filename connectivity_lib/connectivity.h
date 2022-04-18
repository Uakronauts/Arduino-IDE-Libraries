#pragma once

#include "Arduino.h"

#define START_DELIM ' '
#define END_DELIM '~'
#define CHAR_START 32

void writeToBothPorts(const char);

class Tester{
    public:
        Tester();
        Tester(short, long);

        short getNumOfTests();
        long getNumOfDataPoints();

        void setNumOfTests(short);
        void setNumOfDataPoints(long);
    private:
        short numOfTests;
        long numOfDataPoints;
};

class Transmitter: public Tester{
    public:
        Transmitter(short, long);

        bool TransmissionTest();    //put in setup
    private:
        short testNumState = CHAR_START;
        void printTestState(short, short);
};

class Receiver: public Tester{
    public:
        Receiver(short, long);

        void actAsReceiver();   //put in loop
        void printDiagnostics();
    private:
        long* numOfSuccesses;
        long testSuccesses;
        short expectedState = CHAR_START;
};