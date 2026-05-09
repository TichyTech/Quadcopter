#include "definitions.h"

using namespace BLA;

void printVec3(Vector3 v, uint8_t precision);
void printSensorsCalib(Measurements m);
void printVec3commas(Vector3 v, float scale);
void printVec4(Vector4 v, uint8_t precision);
void printVec6(Matrix<6,1> v, uint8_t precision);
void printVec7(Matrix<7,1> v, uint8_t precision);
void printState(State state, uint8_t precision);
void printMeasurement(Measurements m, uint8_t precision);
void printR2Processing(Matrix3 R);
void printV2Processing(Vector3 v);
void printV2ProcessingFloat(Vector3 v);