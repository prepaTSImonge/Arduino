  
// Filtrage passe bas, passe haut et filtre complementaire des angles de la centrale inertielle analogique
// Utilisation d'une interruption

// ********************************************************************************************************************
// --------------------------------BIBLIOTHEQUES ----------------------------------------------------------------------
//(à importer dans Arduino :croquis\importer bibliotheque puis sélection fichier zip)
#include <FlexiTimer2.h>             // Timer 2 utilisé dans la boucle d'asservissement
#include <digitalWriteFast.h>        // Lecture digitale plus rapide

// ********************************************************************************************************************
// ---------------------------------  VARIABLES   ----------------------------------------------------------------------
int gyro_Z_Pin = A3;                          // entrées analogiques utilisées pour les 2 capteurs
int accel_X_Pin = A4;               

int accelerometre_X = 0;                      // valeurs des 2 capteurs
int gyro_Z = 0;

int offset_accX = 0;                          // bias des 2 capteurs
int offset_gyro = 0;

float angle_gyro = 0;                          // angles renvoyés par chacun des 2 capteurs (pour le debug ou filtrage)
float last_angle_gyro = 0;                     // utile pour Euler 
float angle_accelero = 0;
float angle_accelero_prec = 0;    
float angle_gyro_filtrePH = 0;                 // angle filtré passe Haut
float angle_accelero_filtrePB = 0;             // angle filtré passe Bas
float angle_segway_complementaire = 0;         // angle filtré complémentaire


// ****** coefficients de conversion des capteurs ********* :
// Gyroscope ADRX 614(IMU vert) (5000/1024)*(1/25)=0.1953    car sensitivity= 25 mV/°/s
// Accelerometre ADXL 320 (IMU vert) : experim entre -30 et +30 = 1.02 ; 1.4 °/bit =(5000 mV/1024 bits)*(90°/312 mV)
// Accelerometre ADXL203 (IMU rouge) experim entre + et 30°: 0.2918 °/bit =(5000 mV/1024 bits)*(90°/1506mV)
                   
float coeff_accelero = 1;                     
float coeff_gyro = 0.1953;

#define CADENCE_MS 10                        // Temps d'échatillonnage en millisecondes (10 ms soit 100 fois par seconde)
volatile double dt = CADENCE_MS/1000.;       // Période d'échantillonnge en secondes

// Constantes des filtres
float TauPB = 0.2;                           // Constante du filtre passe bas du 1er ordre (modifiable)
float TauPH = 0.2;                           // Constante du filtre passe haut du 1er ordre (modifiable)
// *********************************************************************************************************
// ----------------------- BOUCLE D'INITIALISATION ---------------------------------------------------------
void setup()
{
// ********* A. Configuration port série  *************
    Serial.begin(115200);                                            // configuration de la vitesse de connexion série : 115200 bauds (ou bits par seconde)
    Serial.flush();

// ************ B.Calcul des offsets *********(valeur moyenne de 10 valeurs, capteurs en position initiale : stable et horizontal)
    Serial.println("Calibrage : Positionner le systeme immobile et horizontal... ");
    delay(3000);
    for (int compteur = 0; compteur < 10; compteur++)               // mesure et moyenne de n valeurs    
          {
            accelerometre_X = analogRead (accel_X_Pin);                    
            offset_accX = offset_accX + accelerometre_X;  
          
            gyro_Z = analogRead (gyro_Z_Pin);  
            offset_gyro = offset_gyro + gyro_Z;  
          
            delay (20);  
          }
    offset_accX = round(offset_accX / 10); 
    offset_gyro = round(offset_gyro / 10); 

    Serial.print("offset accelero = "); 
    Serial.println(offset_accX); 
    Serial.print("offset gyro = "); 
    Serial.println(offset_gyro); 
    delay(1000);

// *********** C. Configuration et Lancement de l'interruption ****************
    FlexiTimer2::set(CADENCE_MS, 1/1000., isrt);                                          // période du timer 10 ms, résolution timer = 1 ms , fonction d'interruption isrt
    FlexiTimer2::start();
}
// **************************************************************************************************
// ------------------------------- PROGRAMME PRINCIPAL ----------------------------------------------
// **************************************************************************************************
void loop()
{
// ************************************************************************
// Choisir votre option (en mettant en commentaires l'option non utilisée):
// ************************************************************************
// Option1: Affichage dans le traceur série des valeurs des angles bruts et filtrés

      Serial.print(angle_accelero);
      Serial.print(',');
      Serial.print(angle_accelero_filtrePB); 
      Serial.print(',');      
      Serial.print(angle_gyro);
      Serial.print(',');
      Serial.print(angle_gyro_filtrePH);
      Serial.print(',');
      Serial.print(angle_segway_complementaire);
      Serial.println();

// Option2: Affichage dans le moniteur série des valeurs brutes
/* 
      accelerometre_X = analogRead (accel_X_Pin);
      accelerometre_X =  offset_accX - accelerometre_X;
      Serial.print(accelerometre_X);
      Serial.print(" ; ");
      gyro_Z = analogRead (gyro_Z_Pin);
      gyro_Z =  offset_gyro - gyro_Z;
      Serial.println(gyro_Z);
*/
      delay(10);
}

      
// FONCTION INTERRUPTION -----------------------------------------------------------------------------
void isrt() 
{  
// estimation de l'angle d'inclinaison
  accelerometre_X = analogRead (accel_X_Pin);
  accelerometre_X =  offset_accX - accelerometre_X;
  angle_accelero = float(accelerometre_X / coeff_accelero);
  
  gyro_Z = analogRead (gyro_Z_Pin);
  gyro_Z =  offset_gyro - gyro_Z;
  angle_gyro = angle_gyro + (gyro_Z * coeff_gyro) * dt;                                                        // inutilisé, juste pour l'affichage

// Application de filtres
   // filtre complémentaire
     angle_segway_complementaire = 0.98 * (angle_segway_complementaire + gyro_Z * coeff_gyro * dt) + 0.02 * angle_accelero;              // application filtre complémentaire
   // filtre passe bas pour l'accéléromètre
     angle_accelero_filtrePB = angle_accelero_filtrePB + (dt/TauPB)*(angle_accelero - angle_accelero_filtrePB);
   // filtre passe haut pour le gyroscope
     angle_gyro_filtrePH = angle_gyro_filtrePH*(1-dt/TauPH)+(angle_gyro - last_angle_gyro);
   last_angle_gyro = angle_gyro;     
}
