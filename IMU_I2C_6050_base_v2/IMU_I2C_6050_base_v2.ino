// Calcul basique des angles pour le capteur MPU 6050
// utilisation de la bibliothèque MPU6050

#include "Wire.h"             // Arduino Wire library
#include "I2Cdev.h"           //Installer ces 2 librairies
#include "MPU6050.h"

MPU6050 accelgyro;

int16_t ax, ay, az;                                              //mesures brutes
int16_t gx, gy, gz;
float angle_gyro=0;
float angle_accelero=0;


void setup() {
  Wire.begin();                                                   //I2C bus
  Serial.begin(115200);
  while (!Serial) {;}
  accelgyro.initialize();                                         // initialize device
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  angle_gyro=angle_gyro+float(gz)*0.01/131;                       // 131 cf doc MPU6050 et 0.01=dt en s
  angle_accelero= atan2((double)ax,(double)ay )*180/PI;           // 180/PI permet d'avoir la valeur en °
  Serial.print(angle_accelero);
  Serial.print(","); 
  Serial.println(angle_gyro);  
  delay(10);    
}
