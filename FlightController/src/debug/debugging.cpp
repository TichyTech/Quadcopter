#include "debugging.h"

void printVec3(Vector3 v, uint8_t precision){
  Serial.print(v(0), precision);
  Serial.print(' ');
  Serial.print(v(1), precision);
  Serial.print(' ');
  Serial.print(v(2), precision);
  Serial.print(' ');
}

/**
 * to be used with motionCal.exe which demands these exact scalings for calibration
 */
void printSensorsCalib(Measurements m){
  Serial.print("Raw:");
    printVec3commas(m.acc_vec, 8192.0f);
    Serial.print(',');
    printVec3commas(m.gyro_vec, 16.0f);
    Serial.print(',');
    printVec3commas(m.mag_vec, 1000.0);
    Serial.println();
}

void printVec3commas(Vector3 v, float scale){
  Serial.print(int(v(0)*scale));
  Serial.print(',');
  Serial.print(int(v(1)*scale));
  Serial.print(',');
  Serial.print(int(v(2)*scale));
}

void printVec4(Vector4 v, uint8_t precision){
  Serial.print(v(0), precision);
  Serial.print(' ');
  Serial.print(v(1), precision);
  Serial.print(' ');
  Serial.print(v(2), precision);
  Serial.print(' ');
  Serial.print(v(3), precision);
  Serial.print(' ');
}

void printVec6(Matrix<6,1> v, uint8_t precision){
  for (int i = 0; i < 6; i++){
    Serial.print(v(i), precision);
    Serial.print(' ');
  }
}

void printVec7(Matrix<7,1> v, uint8_t precision){
  for (int i = 0; i < 7; i++){
    Serial.print(v(i), precision);
    Serial.print(' ');
  }
}

void printState(State state, uint8_t precision){
  printVec3(state.pos, precision);
  Serial.print(' ');
  printVec3(state.vel, precision);
  Serial.print(' ');
  printVec3(state.rpy, precision);
  Serial.print(' ');
  printVec3(state.omega, precision);
  Serial.println();

}

void printMeasurement(Measurements m, uint8_t precision){
  Serial.print("gyro: ");
  printVec3(m.gyro_vec, precision);
  Serial.print("acc: ");
  printVec3(m.acc_vec, precision);
  Serial.print("mag: ");
  printVec3(m.mag_vec, precision);
}

void printR2Processing(Matrix3 R){
  Matrix3 M = R * 1024.0f;
  for (int j = 0; j < 3; j++){
    for (int i = 0; i < 3; i++){
      Serial.print(int(M(i,j)));  // col1, col2, col3
      Serial.print(' ');
    }
  }
  Serial.print('\n');
}

void printV2Processing(Vector3 v){
  for (int i = 0; i < 3; i++){
    Serial.print(int(v(i)));
    Serial.print(' ');
  }
  Serial.print('\n');
}

void printV2ProcessingFloat(Vector3 v){
  for (int i = 0; i < 3; i++){
    Serial.print(v(i), 5);
    Serial.print(' ');
  }
  Serial.print('\n');
}
