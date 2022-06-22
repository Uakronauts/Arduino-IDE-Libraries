#include "SoF.h"

// SoF.cpp

float GetAvgValue(float* arr, unsigned int size)
{
  float total = 0.0;
  for (unsigned int i = 0; i < size; ++i)
  {
    total += arr[i];
  }

  return total / size;
}


//Sliiiiiiiiiiiiide to the left
// <--<--<--
// [1, 2, 3] -> [2, 3, 0]
void SlideLeft(float* arr, unsigned int size)
{
  memmove(arr, arr + 1, sizeof(float) * size);
  arr[size - 1] = 0.0;

  return;
}

//Sliiiiiiiiiiiiide to the right (...chris-cross...?)
// -->-->-->
// [1, 2, 3] -> [0, 1, 2]
void SlideRight(float* arr, unsigned int size)
{
  memmove(arr + 1, arr, sizeof(float) * size);
  arr[0] = 0.0;

  return;
}

void Prepend(float val, float* arr, unsigned int size)
{
  SlideRight(arr, size);
  arr[0] = val;

  return;
}



float GetRange(float* arr, unsigned int size)
{
  float min = std::numeric_limits<float>::max();
  float max = std::numeric_limits<float>::min();
  for (unsigned int i = 0; i < size; ++i)
  {
    if (arr[i] > max)
    {
      max = arr[i];
    }

    if (arr[i] < min)
    {
      min = arr[i];
    }
  }

  return max - min;
}

// Constructors

// Base Constructor
SoF::SoF(float* newAlt, float* newAccel, float* newGyro, float* newMag, float* newEuler, float* newQuat) {
  SetAltVar(newAlt);
  SetAccelArr(newAccel);
  SetGyroArr(newGyro);
  SetMagArr(newMag);
  SetEulerArr(newEuler);
  SetQuatArr(newQuat);

  currentState = States::pad;
  deltaTime = (float)1.0 / LOOP_REFRESHRATE;
  storedAccelDown = storedAccelY;
  isDownInverted = false;
}

SoF::SoF(float* newAlt, float* newAccel, float* newGyro, float* newMag, float* newEuler, float* newQuat, unsigned int newRefreshRateHz) {
  SetAltVar(newAlt);
  SetAccelArr(newAccel);
  SetGyroArr(newGyro);
  SetMagArr(newMag);
  SetEulerArr(newEuler);
  SetQuatArr(newQuat);

  currentState = States::pad;
  LOOP_REFRESHRATE = newRefreshRateHz;
  deltaTime = (float)1.0 / LOOP_REFRESHRATE;
  storedAccelDown = storedAccelY;
  isDownInverted = false;
}

SoF::SoF(float* newAlt, float* newAccel, float* newGyro, float* newMag, float* newEuler, float* newQuat, bool downInverted) {
  SetAltVar(newAlt);
  SetAccelArr(newAccel);
  SetGyroArr(newGyro);
  SetMagArr(newMag);
  SetEulerArr(newEuler);
  SetQuatArr(newQuat);

  currentState = States::pad;
  deltaTime = (float)1.0 / LOOP_REFRESHRATE;
  storedAccelDown = storedAccelY;
  isDownInverted = downInverted;
}

SoF::SoF(float* newAlt, float* newAccel, float* newGyro, float* newMag, float* newEuler, float* newQuat, bool downInverted, unsigned int newRefreshRateHz) {
  SetAltVar(newAlt);
  SetAccelArr(newAccel);
  SetGyroArr(newGyro);
  SetMagArr(newMag);
  SetEulerArr(newEuler);
  SetQuatArr(newQuat);

  currentState = States::pad;
  LOOP_REFRESHRATE = newRefreshRateHz;
  deltaTime = (float)1.0 / LOOP_REFRESHRATE;
  storedAccelDown = storedAccelY;
  isDownInverted = downInverted;
}

//Refresh CMD
void SoF::Refresh() {
  // Function Re-Store
  Prepend(*altVar, storedAlts, STORELEN);

  Prepend(accelArr[0], storedAccelX, STORELEN);
  Prepend(accelArr[1], storedAccelY, STORELEN);
  Prepend(accelArr[2], storedAccelZ, STORELEN);

  Prepend(gyroArr[0], storedGyroX, STORELEN);
  Prepend(gyroArr[1], storedGyroY, STORELEN);
  Prepend(gyroArr[2], storedGyroZ, STORELEN);

  Prepend(magArr[0], storedMagX, STORELEN);
  Prepend(magArr[1], storedMagY, STORELEN);
  Prepend(magArr[2], storedMagZ, STORELEN);

  // Function recalc derivatives
  derivAlt = recalculateDerivative(storedAlts);

  Prepend(derivAlt, storedAltDerivs, STORELEN);
  deriv2Alt = recalculateDerivative(storedAltDerivs);

  derivAccelX = recalculateDerivative(storedAccelX);
  derivAccelY = recalculateDerivative(storedAccelY);
  derivAccelZ = recalculateDerivative(storedAccelZ);

  derivGyroX = recalculateDerivative(storedGyroX);
  derivGyroY = recalculateDerivative(storedGyroY);
  derivGyroZ = recalculateDerivative(storedGyroZ);

  derivMagX = recalculateDerivative(storedMagX);
  derivMagY = recalculateDerivative(storedMagY);
  derivMagZ = recalculateDerivative(storedMagZ);

  SporaticFactor_Accel_X = GetRange(storedAccelX, STORELEN);
  SporaticFactor_Accel_Y = GetRange(storedAccelY, STORELEN);
  SporaticFactor_Accel_Z = GetRange(storedAccelZ, STORELEN);

  // Function determine SoF w/ Methods
  if (IsAscent())
  {
    if (IsBooster())
    {
      if (IsBoost())
      {
        currentState = States::booster_boost;
      }
      else
      {
        currentState = States::booster_coast;
      }
    }
    // If trying to transition from coast -> boost, switch to sustainer
    else if ((currentState == States::booster_coast && IsBoost()) ||
      (currentState == States::sustainer_boost || currentState == States::sustainer_coast) ||
      IsSustainer())
    {
      if (IsBoost())
      {
        currentState = States::sustainer_boost;
      }
      else  //IsCoast
      {
        currentState = States::sustainer_coast;
      }
    }
    // If here -- EDGE CASE REACHED!! Print here in debug output.
  }
  else if (IsDescent())
  {
    if (IsMainDeployed())
    {
      currentState = States::main;
    }
    else if (currentState != States::main) //drogue deployed (don't go backwards)
    {
      currentState = States::drogue;
    }
  }
  else if (IsStableState())
  {
    if (IsOnPad())
    {
      currentState = States::pad;
    }
    else if (*altVar > 3000.0)
    {
      currentState = States::apogee;
    }
    else
    {
      currentState = States::ground;
    }
  }



}

//Refresh CMD
void SoF::Refresh(bool debug) {

}

void SoF::DebugOutput() {

}

float SoF::recalculateDerivative(float* storageArr)
{
  return (storageArr[0] - storageArr[STORELEN - 1]) / ((float)STORELEN - 1.0 / LOOP_REFRESHRATE);
}

float SoF::conditionalInvertAccelDown(float accDown)
{
  if (!isDownInverted)
  {
    return accDown;
  }

  return accDown * -1; //invert accDown if isDownInverted is 1 (true)
}

// Setters
void SoF::SetAltVar(float* newAlt) {
  altVar = newAlt;
}

void SoF::SetAccelArr(float* newAccel) {
  accelArr = newAccel;
}

void SoF::SetGyroArr(float* newGyro) {
  gyroArr = newGyro;
}

void SoF::SetMagArr(float* newMag) {
  magArr = newMag;
}

void SoF::SetEulerArr(float* newEuler) {
  eulerArr = newEuler;
}

void SoF::SetQuatArr(float* newQuat) {
  quatArr = newQuat;
}

bool SoF::SetDownDir(unsigned int downDir)
{
  if (downDir > 2) {
    return false;
  }

  else if (downDir == 0)
  {
    storedAccelDown = storedAccelX;
  }
  else if (downDir == 1)
  {
    storedAccelDown = storedAccelY;
  }
  else if (downDir == 2)
  {
    storedAccelDown = storedAccelZ;
  }

  return true;
}

// Getters
States SoF::getState() {
  return currentState;
}

const char** SoF::getStateVals() {
  return StateVals;
}


//Methods
const char* SoF::CurrentState() {
  return StateVals[(int)currentState];
}

bool SoF::IsStableState() {  //pad or ground
  // If Ascent and Descent are designed properly, the gap between their tolerances
  return (!IsAscent() && !IsDescent());
}

bool SoF::IsAscent() {
  //delta altitude > 0
  // accel down > grav (by a lot, with some wiggle room)

  if (GetAvgValue(storedAltDerivs, STORELEN) > 1.0 * DER_ALT_ASCENT_TOLERANCE_FACTOR)
  {
    return true;
  }

  return false;
}

bool SoF::IsDescent() {
  if (GetAvgValue(storedAltDerivs, STORELEN) < -1.0 * DER_ALT_DESCENT_TOLERANCE_FACTOR)
  {
    return true;
  }

  return false;
}

bool SoF::IsOnPad() {
  if (IsStableState())
  {
    float avgAccelDown = conditionalInvertAccelDown(GetAvgValue(storedAccelDown, 5));

    if (avgAccelDown < 9.8 * ACCELDOWN_TOLERANCE_FACTOR &&
      avgAccelDown > 9.8 * (1 / ACCELDOWN_TOLERANCE_FACTOR))
    {
      return true;
    }
  }
  return false;
}


bool SoF::IsBoost() {
  if (IsAscent())
  {
    // If accel down is > 1G, it is DEFINITELY boost... otherwise
    if (conditionalInvertAccelDown(storedAccelDown[0]) > 9.8 * ACCELDOWN_TOLERANCE_FACTOR ||
      conditionalInvertAccelDown(GetAvgValue(storedAccelDown, 7)) > 9.8 * ACCELDOWN_TOLERANCE_FACTOR)
    {
      return true;
    }
  }
  return false;
}

bool SoF::IsCoast() {
  return !IsBoost();  // i mean... this makes sense to me!
}


bool SoF::IsBooster() {
  if (IsAscent())
  {
    if (GetAvgValue(storedAlts, 5) > MAX_BOOSTER_HEIGHT)
    {
      return false;
    }
    return true;
  }
  return false;
}

bool SoF::IsSustainer() {
  if (IsAscent())
  {
    if (GetAvgValue(storedAlts, 5) < MIN_SUSTAINER_HEIGHT)
    {
      return false;
    }
    return true;
  }
  return false;
}


bool SoF::IsDrogueDeployed() {
  //more reliable than using consensus system since the acceleration could be different by rocket weight
  return IsDescent(); 
}

bool SoF::IsMainDeployed() {
  if (IsDescent())
  {
    short consensus = 0;
    consensus += IsWithinSFCategory(SporaticFactor_Accel_X, MAIN_ACCEL_SF, MAIN_ACCEL_SF_TOLERANCE) +
      IsWithinSFCategory(SporaticFactor_Accel_Y, MAIN_ACCEL_SF, MAIN_ACCEL_SF_TOLERANCE) +
      IsWithinSFCategory(SporaticFactor_Accel_Z, MAIN_ACCEL_SF, MAIN_ACCEL_SF_TOLERANCE);

    if (consensus > 1)
    {
      return true;
    }
  }
  return false;
}

bool SoF::IsWithinSFCategory(float SF, float DesiredSF, float PercTol)
{
  return (SF < (DesiredSF + (DesiredSF * PercTol)) && SF > (DesiredSF - (DesiredSF * PercTol)));
}