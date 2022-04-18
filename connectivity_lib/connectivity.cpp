#include "connectivity.h"

void writeToBothPorts(const char ch)
{
    Serial.print(ch);
    Serial1.print(ch);
}

Tester::Tester(){
    numOfTests = 1;
    numOfDataPoints = 10;
}

Tester::Tester(short tests, long points){
    numOfTests = tests;
    numOfDataPoints = points;
}

short Tester::getNumOfTests(){
    return numOfTests;
}

long Tester::getNumOfDataPoints(){
    return numOfDataPoints;
}

void Tester::setNumOfTests(short tests){
    if(tests > 90){
        Serial.print("BALLS");
        while(1);
    }
    numOfTests = tests;
}

void Tester::setNumOfDataPoints(long points){
    numOfDataPoints = points;
}

Transmitter::Transmitter(short tests, long points){
    setNumOfTests(tests);
    setNumOfDataPoints(points);
}

bool Transmitter::TransmissionTest(){
    Serial.println("TRANSMISSION TEST BEGUN");
    ++testNumState;
    while(testNumState <= getNumOfTests() + CHAR_START){
        printTestState(testNumState - CHAR_START, getNumOfTests());
        for(size_t i = 0; i < getNumOfDataPoints(); ++i){
            writeToBothPorts((char)testNumState);
        }
        Serial.println();
        ++testNumState;
    } 
    return true;
}

void Transmitter::printTestState(short curr, short total){
    Serial.print("Test State: ");
    Serial.print(curr);
    Serial.print('/');
    Serial.print(total);
    Serial.print(": ");
}

Receiver::Receiver(short tests, long points){
    setNumOfTests(tests);
    setNumOfDataPoints(points);

    numOfSuccesses = new long(tests);
}

void Receiver::actAsReceiver()
{
    if(Serial.available())
    {
        char var = Serial.read();
        if(var == expectedState){
            ++testSuccesses;
        }
        else if(var == START_DELIM)
            ++expectedState;
        else if(var == END_DELIM){
            numOfSuccesses[expectedState - START_DELIM - 1] = testSuccesses;
            testSuccesses = 0;
        }
        else if(expectedState > getNumOfTests()){
            printDiagnostics();
            while(1);
        }
        else{
            Serial.print('X');
        }
    }
}

void Receiver::printDiagnostics(){
    for(int balls = 0; balls < getNumOfTests(); ++balls){
        Serial.print("Sequence ");
        Serial.print(balls);
        Serial.print(": ");
        Serial.print((float)numOfSuccesses[balls]/getNumOfDataPoints() * 100);
        Serial.println('%');
    }
}