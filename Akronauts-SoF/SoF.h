#pragma once

#include <cstring>
#include <limits>

//Array helpers.

float GetAvgValue(float*, unsigned int);

//Sliiiiiiiiiiiiide to the left
// <--<--<--
// [1, 2, 3] -> [2, 3, 0]

void SlideLeft(float*, unsigned int);

//Sliiiiiiiiiiiiide to the right (...chris-cross...?)
// -->-->-->
// [1, 2, 3] -> [0, 1, 2]
void SlideRight(float* arr, unsigned int);

void Prepend(float, float*, unsigned int);

float GetRange(float*, unsigned int);

//SoF.hpp

enum class States
{
  pad = 0,

  // BOOSTER
  booster_boost,
  booster_coast,

  // SUSTAINER
  sustainer_boost,
  sustainer_coast,

  apogee,

  // CHUTES
  drogue,
  main,

  ground
};

// SoF CLASS

class SoF {
private:
  //Current state
  States currentState;


  // StoreLen will detemine how long the derivative change of time will be.
  // Derivative functions take the most recent and least recent data points.
  // Ex: STORELEN of 5 will take a derivative across every 0.5 seconds, but derivatives
  // will be recalculated every (1 / LOOP_REFRESHRATE) seconds
  static const unsigned int STORELEN = 15;
  unsigned int LOOP_REFRESHRATE = 10; //set to 10 by default
  bool isDownInverted;

  const float ALT_TOLERANCE_FACTOR = 1.0F;
  const float DER_ALT_ASCENT_TOLERANCE_FACTOR = 0.1F;
  const float DER_ALT_DESCENT_TOLERANCE_FACTOR = 1.15F;

  const float ACCELX_TOLERANCE_FACTOR = 1.0;
  const float ACCELY_TOLERANCE_FACTOR = 1.0;
  const float ACCELZ_TOLERANCE_FACTOR = 1.0;

  const float ACCELDOWN_TOLERANCE_FACTOR = 1.75F;

  const float MAX_BOOSTER_HEIGHT = 12000;
  const float MIN_SUSTAINER_HEIGHT = 9600;

  float SporaticFactor_Accel_X;
  float SporaticFactor_Accel_Y;
  float SporaticFactor_Accel_Z;

  const float DROGUE_ACCEL_SF = 20.0;
  const float DROGUE_ACCEL_SF_TOLERANCE = 0.05;  //% diff

  const float MAIN_ACCEL_SF = 6.0;
  const float MAIN_ACCEL_SF_TOLERANCE = 0.20;  //% diff

  bool IsWithinSFCategory(float, float, float); //factor, wanted state, %tolerance

  // Altitude FROM ground level in feet
  float* altVar;
  float storedAlts[STORELEN];

  float derivAlt;
  float storedAltDerivs[STORELEN];

  float deriv2Alt;

  // Time
  // float* timeVar;
  // float storedTimes[STORELEN];
  float deltaTime;

  // Pointers to the data arrays for accel, gyro, and mag
  float* accelArr,
    * gyroArr,
    * magArr;

  // Accel
  float storedAccelX[STORELEN];
  float derivAccelX;

  float storedAccelY[STORELEN];
  float derivAccelY;

  float storedAccelZ[STORELEN];
  float derivAccelZ;

  float* storedAccelDown;

  // Gyro
  float storedGyroX[STORELEN];
  float derivGyroX;

  float storedGyroY[STORELEN];
  float derivGyroY;

  float storedGyroZ[STORELEN];
  float derivGyroZ;

  // Mag
  float storedMagX[STORELEN];
  float derivMagX;

  float storedMagY[STORELEN];
  float derivMagY;

  float storedMagZ[STORELEN];
  float derivMagZ;

  // Pointer for euler vals
  float* eulerArr;

  // Pointer for quaternion val
  float* quatArr;

  const char* StateVals[9] = { "PAD", "BOOSTER_BOOST", "BOOSTER_COAST", "SUSTAINER_BOOST", "SUSTAINER_COAST", "APOGEE", "DROGUE", "MAIN", "GROUND" };

  void DebugOutput();
  float recalculateDerivative(float*);  //Takes a storage array as arg and calculates a derivative with first and last data points. This will be garbage for the first STORELEN times.
  float conditionalInvertAccelDown(float);

public:
  // Constructors
  SoF(float* newAlt, float* newAccel, float* newGyro, float* newMag, float* newEuler, float* newQuat);
  SoF(float* newAlt, float* newAccel, float* newGyro, float* newMag, float* newEuler, float* newQuat, unsigned int newRefreshRateHz);
  SoF(float* newAlt, float* newAccel, float* newGyro, float* newMag, float* newEuler, float* newQuat, bool downInverted);
  SoF(float* newAlt, float* newAccel, float* newGyro, float* newMag, float* newEuler, float* newQuat, bool downInverted, unsigned int newRefreshRateHz);;


  // Destructors shouldn't be needed, should not dynamically allocate this object

  //Refresh CMD
  void Refresh();
  void Refresh(bool debug);

  // Setters
  void SetAltVar(float*);
  void SetAccelArr(float*);
  void SetGyroArr(float*);
  void SetMagArr(float*);
  void SetEulerArr(float*);
  void SetQuatArr(float*);
  bool SetDownDir(unsigned int);  //0:X, 1:Y, 2:Z

  // Getters
  States getState();
  const char** getStateVals();

  //Methods
  const char* CurrentState();

  bool IsStableState();  //pad or ground
  bool IsAscent();
  bool IsDescent();

  bool IsOnPad();

  bool IsBoost();
  bool IsCoast();

  bool IsBooster();
  bool IsSustainer();

  bool IsDrogueDeployed(); //this just needs to return true for descent since drogue is always deployed after app
  bool IsMainDeployed();
};

