// Filtrage passe bas , passe haut et complémentaire pour le capteur MPU 6050
// Plages de fonctionnement réglables
// --------------------- NOTA --------------------------//
// Axe X horizontal dans le sens du déplacement - Axe Y vertical vers le haut 
// Angle mesuré par rappport à la verticale (Y)
/*    ________   Capteur :
     |        |
     |   Y    |
     |   |    |
     |   |___ |
     |     X  |
     |________|
*/
// Bibliothèques utiles
#include "Wire.h"                           // bibliothèque I2C
#include "I2Cdev.h"                         // Installer ces 2 librairies
#include "MPU6050.h"

MPU6050 mpu;                                 // declaration d'un objet

// Coefficients des capteurs (dépendant du réglage des registres : confer fonction setting_sensor() dans le Setup)
float    GYRO_FACTOR;
float    ACCEL_FACTOR;

//  Offset (ou bias) des capteurs
float    offset_gyro_z = 0;
float    offset_x_accel = 0;
float    offset_y_accel = 0;
float    offset_z_accel = 0;

// Mesures et Angles
int16_t ax, ay, az;                            // mesures brutes
int16_t gx, gy, gz;
float angle_gyro=0;
float angle_accelero=0;
float angle_gyro_filtrePH = 0;                 // angle filtré passe Haut
float angle_accelero_filtrePB = 0;             // angle filtré passe Bas
float angle_filtre_complementaire=0;

// valeurs utiles aux capteurs
unsigned long last_read_time;
float    last_angle_accelero;
float    last_angle_gyro;

// Constantes des filtres
float TauPB = 0.2;                           // Constante du filtre passe bas du 1er ordre (modifiable)
float TauPH = 0.2;                           // Constante du filtre passe haut du 1er ordre (modifiable)

const float RADIANS_TO_DEGREES = 57.2958;                 // 180/pi
// **********************************************************************************************
void setup() {
      Wire.begin();                                         //I2C bus
      Serial.begin(115200);
      mpu.initialize();                                     // initialize device
      Serial.println("Test de la connection du dispositif ...");
      Serial.println(mpu.testConnection() ? "MPU6050 connection reussie" : "MPU6050 connection echec");
    
      setting_sensors();                                    // Réglage des plages de fonctionnement via les registres  
      calibrate_sensors();                                  // Mesurage des offsets des capteurs
      
      delay(1000);
      last_read_time = millis();
}

void loop() {
// *********************** 1. Lecture **************************************
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);                   // lecture complète du capteur I2C

      unsigned long t_now = millis();                                 // temps de la dernière lecture

// *********************** 2. Calculs **************************************
// offsets et gain pour le gyroscope  
      float gyro_z = (gz - offset_gyro_z)/GYRO_FACTOR;                                  // en °/s

// valeurs accelerometre (modifs signe possible en fonction de l'implantation)
      float accel_x = ax;                                             
      float accel_y = ay;                                             
      float accel_z = az; 
     
// Calcul des angles bruts via les accéléromètres              
      angle_accelero = atan(ax/sqrt(pow(ay,2) + pow(az,2)))*RADIANS_TO_DEGREES;    // par rapport à l'axe Y

// Calcul des angles bruts via les gyroscopes
      float dt =(t_now - last_read_time)/1000.0;                                        // en secondes
      angle_gyro = gyro_z*dt + last_angle_gyro;

// calcul des angles filtrés 
      // Passe Bas pour l'accéléromètre
      angle_accelero_filtrePB = angle_accelero_filtrePB + (dt/TauPB)*(angle_accelero - angle_accelero_filtrePB);

      // Passe Haut pour le gyroscope
      angle_gyro_filtrePH = angle_gyro_filtrePH*(1-dt/TauPH)+(angle_gyro - last_angle_gyro);

// Application du filtre complémentaire
      const float alpha = 0.98;                                                         //alpha dépend du temps d'echantillonnage
      angle_filtre_complementaire = alpha*(angle_filtre_complementaire + gyro_z*dt) + (1.0 - alpha)*angle_accelero;


// ***************** 3. Affichages (Décommenter les affichages utiles) ******************
// angles via l'accelero
      Serial.print(angle_accelero);
      Serial.print(","); 

// angles via le gyro 
      Serial.print(angle_gyro);  
      Serial.print(",");

// angles filtrés de l'accéléro et gyro 
      Serial.print(angle_accelero_filtrePB);
      Serial.print(",");
      Serial.print(angle_gyro_filtrePH);
      Serial.print(",");

// angles via le filtre complémentaire 
      Serial.print(angle_filtre_complementaire);
 
      Serial.println();


// ****************************4. Opérations de boucle ************************************
// Delai de boucle (modifiable mais lié à alpha) 
      delay(10);

// Enregistrement des dernieres valeurs pour la boucle suivante
      last_read_time = t_now;
      last_angle_accelero = angle_accelero;
      last_angle_gyro = angle_gyro;    
}


// ================================================================
// ===                Fonctions utiles                      ===
// ================================================================
// Calibration initial(offset ou bias du capteur)
void calibrate_sensors() {
  int  num_readings = 20;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  for (int i = 0; i < num_readings; i++) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        offset_gyro_z += gz;
        offset_x_accel += ax;
        offset_y_accel += ay;
        offset_z_accel += az;
  }
  
  offset_gyro_z /= num_readings;
  offset_x_accel /= num_readings;
  offset_y_accel /= num_readings;
  offset_z_accel /= num_readings;
}

//*************************************************************************************
// Choix des plages de fonctionnement
void setting_sensors() {

// *************** Gyroscope **************************    
// Solution 1 : modification dans le programme
    uint8_t FS_SEL = 0;                                                             // modifiable entre 0 et 3: confer datasheet MPU 6050
// Solution 2 : entrée au clavier (à decommenter si besoin)
/*
    Serial.println("Reglages du gyroscope");
    Serial.println("Taper 0 pour une plage de 250°/s");
    Serial.println("Taper 1 pour une plage de 500°/s");
    Serial.println("Taper 2 pour une plage de 1000°/s");
    Serial.println("Taper 3 pour une plage de 2000°/s");
                                     
    while (Serial.available()==0){}                                                    // attente de valeur ou depart
    while (Serial.available() > 0)                                                     // tant que quelquechose est en attente sur le port série
          {FS_SEL = Serial.parseInt();}
*/
// *******************************************                
    mpu.setFullScaleGyroRange(FS_SEL);                        // Set the full scale range of the gyro

    // Affichage (optionnel) (à decommenter si besoin)
    uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();        // get scale value of gyro
    GYRO_FACTOR = 131.0/(FS_SEL + 1);
/*    
    Serial.println();
    Serial.print("valeur du registre FS_SEL (range) du Gyro = ");
    Serial.print(READ_FS_SEL);
    Serial.print("  ;  plage +/- : ");
          if (READ_FS_SEL==0) {Serial.print(" 250°/s ");}
          else if (READ_FS_SEL==1) {Serial.print(" 500°/s ");}
          else if (READ_FS_SEL==2) {Serial.print(" 1000°/s ");}
          else if (READ_FS_SEL==3) {Serial.print(" 2000°/s ");}
    Serial.print(" ; gain du Gyro = ");
    Serial.println(GYRO_FACTOR);
    Serial.println();
*/
// ***************** Acceleromètre ************************    
// Solution 1 : modification dans le programme
    uint8_t AFS_SEL = 0;                                       // modifiable entre 0 et 3: confer datasheet MPU 6050
// Solution 2 : entrée au clavier (à decommenter si besoin)
/*
    Serial.println("Reglages de l'accélérometre");
    Serial.println("Taper 0 pour une plage de 2g");
    Serial.println("Taper 1 pour une plage de 4g");
    Serial.println("Taper 2 pour une plage de 8g");
    Serial.println("Taper 3 pour une plage de 16g");
                                     
    while (Serial.available()==0){}                                                    // attente de valeur ou depart
    while (Serial.available() > 0)                                                     // tant que quelquechose est en attente sur le port série
          {AFS_SEL = Serial.parseInt();}
*/
// *******************************************                
    mpu.setFullScaleAccelRange(AFS_SEL);                                               // Set the full scale range of the gyro
  
   
// Affichage (optionnel)   (à decommenter si besoin)  
    uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();                                // get default full scale value of accelerometer
    ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);
/*
    Serial.println();
    Serial.print("valeur du registre FS_SEL (range) de l'accelerometre = ");
    Serial.print(READ_AFS_SEL);
    Serial.print(" ;  plage +/- : ");
          if (READ_AFS_SEL==0) {Serial.print(" 2g ");}
          else if (READ_AFS_SEL==1) {Serial.print(" 4g ");}
          else if (READ_AFS_SEL==2) {Serial.print(" 8g ");}
          else if (READ_AFS_SEL==3) {Serial.print(" 16g ");}
    Serial.print(" ; gain de l'accelerometre = ");
    Serial.println(ACCEL_FACTOR);
    Serial.println();
 delay(2000); 
*/
}

