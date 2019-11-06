// Filtrage complémentaire basique pour le capteur MPU 6050
// sans calibration

#include "Wire.h"             // Arduino Wire library
#include "I2Cdev.h"           //Installer ces 2 librairies
#include "MPU6050.h"

MPU6050 accelgyro;

int16_t ax, ay, az;                                     //mesures brutes
int16_t gx, gy, gz;
float angle_gyro=0;
float angle_accelero=0;
float angle_complementaire=0;

void setup() {
  Wire.begin();                                         //I2C bus
  Serial.begin(115200);
  accelgyro.initialize();                               // initialize device
  Serial.println("Test de la connection du dispositif ...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection reussie" : "MPU6050 connection echec");
  delay(1000);

}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  angle_gyro=angle_gyro+float(gz)*0.01/131;                       // 131 cf doc MPU6050 et 0.01=dt en s
  angle_accelero = atan2((double)ax,(double)ay )*180/PI;          // 180/PI permet d'avoir la valeur en °
  angle_complementaire =0.98*(angle_complementaire+float(gz)*0.01/131) + 0.02*atan2((double)ax,(double)ay)*180/PI;
  Serial.print(angle_accelero);
  Serial.print(","); 
  Serial.print(angle_gyro);
  Serial.print(","); 
  Serial.println(angle_complementaire);    
  delay(10);    
}


