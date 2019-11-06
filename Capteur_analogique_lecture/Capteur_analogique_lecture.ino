/*
EXEMPLE de LECTURE DE CAPTEUR ANALOGIQUE
A modifier ou mettre en commentaires en fonction de vos besoins
CPGE TSI Monge
*/
//**************************************
// bibliothèques eventuelles
//#include <nom_bibliotheque.h>                                         // fonctions de haut niveau spécifiques au capteur mises à disposition

//****************************************
// Declaration des variables, constantes et alias
#define sensor_Pin A0                                                    // ou int sensorPin = A0;     Alias avec capteur sur port 0

float sensor_Coef = 0.2;                                                // attention au type de variable utilisée (entiers, réel): voir doc arduino sur int, float)
float sensor_Offset ;
int sensor_Value;                                                       // sur 10 bits 
float filtered_Value=0;

float distance=0;                                                       // variables de sortie

float tau=0.50;                                                         // coefficient d'un filtre éventuel
int dt = 50;                                                            // delai de boucle
const float a1 = -0.08;                                                 // coefficients du capteur
const float b1 = 47;                                                    // constantes occupent moins de place mémoire

//****************************************
// INITIALISATION
void setup() 
  {
              
          //****************************************
          // a. paramétrage du port série
            Serial.begin(115200);                                                 // vitesse de transmission sur port série

          //****************************************
          // b. Etalonnage (facultatif): exemple du calcul de l'offset sur une moyenne de 10 mesures

                int somme = 0;                                                    // variable déclarée localement car non utilisée par la suite  
                for (int i = 0; i < 10 ;i++)                                      // boucle répétée 10 fois ; i déclaré localement
                  {
                  sensor_Value=analogRead(sensor_Pin);
                  somme = somme + sensor_Value;
                  delay(100);                                                     // temporisation en millisecondes
                  } 
                 
                  sensor_Offset =somme/10.0;                                       // attention à mettre en réel le diviseur
                  Serial.print("la valeur moyenne de l'offset est: ");             // affichage sans saut à la ligne
                  Serial.println(sensor_Offset);                                   // affichage avec saut à la ligne
                  delay(3000); 

          //****************************************
          // c. En cas de besoin, on peut utiliser une sortie numérique pour alimenter le capteur en 5V
          pinMode(8, OUTPUT);
          digitalWrite(8, HIGH);

  }                                                                       // Bien VERIFIER que cette accolade ferme le setup
//*****************************************************************************************************************************



//*****************************************************************************************************************************
// PROGRAMME PRINCIPAL
void loop() {

          //**********************
          // 1.Simple lecture
                sensor_Value = analogRead(sensor_Pin);
          
          //****************************************
          // 2a. Application d'une loi avec offset
                //distance = (sensor_Value - sensor_Offset)*sensor_Coef;               // exemple
          
          // 2.b Application d'une autre loi (linéaire ou non...)
                distance = a1*sensor_Value + b1;                                      // exemple

          
           //****************************************
          // 3.Filtrage éventuel (pour un delai de boucle faible)
                 filtered_Value=filtered_Value+(sensor_Value-filtered_Value)*(dt/(1000*tau));       // exemple de filtrage passe bas        
          
          //****************************************
          // 4.test éventuel (saturation,...)
                /*
                if(sensor_Value > 550)
                  {Serial.println("trop proche");
                   distance = 5;}                                                     // valeur par défaut
                */
          //****************************************
          // 5.Affichage / Tracés
          // Moniteur série
                /*  
                Serial.println(sensor_Value);
                Serial.print("la distance est de :");
                Serial.println(distance);                                             // exemple
                */
                
          // Tracé dans le traceur série

                Serial.print(sensor_Value);                                           // Pour afficher plusieurs valeurs dans le traceur série, les données doivent être séparées par une virgule...
                Serial.print(",");
                Serial.println(filtered_Value);                                       // ... et se terminer par un saut de ligne                                                    


          //****************************************
          // 6.Fin de boucle / Temporisation                 
                delay(dt);                                            // attention, le temps de boucle est plus grand notamment à cause des print (solution apprx.). 
                //Pour un temps précis, utiliser une interruption ou lecture du timer
              //  while(1){}                                          // si besoin, boucle infinie pour éviter de reboucler                                                                  

}                                                                     // Bien VERIFIER que cette accolade ferme le LOOP



