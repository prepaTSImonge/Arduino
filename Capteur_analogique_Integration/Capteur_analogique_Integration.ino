/*
EXEMPLE de LECTURE DE CAPTEUR ANALOGIQUE avec integration (soit dans le loop soit par interruption)
A modifier ou mettre en commentaires en fonction de vos besoins
*/
//**************************************
// bibliothèques eventuelles
//#include <nom_bibliotheque.h>
#include <FlexiTimer2.h>                                                // bibliothèque pour gérer une interruption (utile pour une intégration)
//****************************************
// Declaration des variables, constantes et alias
float sensor_Coef = 0.2;                                                // attention au type de variable utilisée (entiers, réel): voir doc arduino sur int, float)
float sensor_Offset ;
int sensor_Value;
float filtered_Value;                                                   // pour le loop
float filtered_Value2;                                                  // pour l'interruption

float distance1=0;                                                       // variables de boucle
float filtered_distance1=0;
float last_distance1=0;
float vitesse1=0;

float distance2=0;                                                       // variables d'interruption
float filtered_distance2=0;
float last_distance2=0;
float vitesse2=0;

float tau=0.50;                                                         // coefficient d'un filtre éventuel
int dt_loop = 50;                                                       // delai de boucle en millisecondes
int dt_ms= 100;                                                         // période d'interruption en millisecondes
float dt = dt_ms/1000.;                                                 // période d'interruption en secondes
                                                       
unsigned long current_time;
unsigned long last_time;
unsigned long last_time1;

const float a1 = -0.08;                                                 // coefficients du capteur
const float b1 = 47;                                                    // constantes occupent moins de place mémoire

#define sensor_Pin 0                                                    // Alias avec capteur sur port 0


//****************************************
// INITIALISATION
void setup() 
  {
       Serial.begin(115200);                                                 // vitesse de transmission sur port série
// Paramétrage de l'interruption
       FlexiTimer2::set(dt_ms, 1/1000., isrt);                   // lance la fonction isrt (en fin de programme) toutes les Te secondes : utile en cas d'intégration (gyroscope, accéleromètre)
       FlexiTimer2::start();                                     // lancement de l'interruption après le calcul d'offset
  }                                                                       
//***************************************

//****************************************
// PROGRAMME PRINCIPAL
void loop() {
// Mesure distance
        sensor_Value = analogRead(sensor_Pin);
        if(sensor_Value > 550){distance1 = 5;}   
        else {distance1 = a1*sensor_Value + b1;}                                      
// Filtrage grossier
        filtered_distance1=filtered_distance1+(distance1-filtered_distance1)*(dt_loop/(1000*tau));
// calcul de vitesse dans la boucle           
        current_time= millis();
        vitesse1 = 1000*(filtered_distance1 - last_distance1)/(current_time - last_time1);        // en cm/s

// Affichage toutes les n millisecondes

        //Serial.print(filtered_distance1);
        //Serial.print(",");
        Serial.print(vitesse1);
        Serial.print(",");
        Serial.println(vitesse2);

// Record for next loop            
        last_distance1 = distance1;
        last_time1 = current_time;
        
// Délai de boucle 
        delay(dt_loop);                                           
}                                                                     


// FONCTION INTERRUPTION -----------------------------------------------------------------------------
void isrt() 
{  
// Exemple de calcul de vitesse
  sensor_Value = analogRead (sensor_Pin);
  distance2 = a1*sensor_Value + b1;
  filtered_distance2=filtered_distance2+(distance2-filtered_distance2)*(dt/tau);
  vitesse2 = (filtered_distance2 - last_distance2)/dt;            // en m/s
  last_distance2 = distance2;
}


