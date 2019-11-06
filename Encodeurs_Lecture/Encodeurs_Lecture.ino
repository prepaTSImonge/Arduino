// Lecture de position et vitesse pour encodeurs optiques
// CPGE TSI monge
//****************************************************************************************
// ***************************** BiIBLIOTHEQUES *****************************************
#include <FlexiTimer2.h>
#include <digitalWriteFast.h>              // bibliothèque lecture rapide entrées digitales

// *****************************  PARAMETRAGE *******************************************
// Adresse des encodeurs
#define encoder_D_PinA  18        // jaune 
#define encoder_D_PinB  19        // blanc
#define encoder_G_PinA  20        // jaune      
#define encoder_G_PinB  21        // blanc 

// ATTENTION à bien paramétrer la(les) fonction(s) attachInterrupt dans le setup suivant les n° suivants:
// Pin 2 pour interruption 0     ou    Pin 21 pour interruption 2     ou      Pin 18 pour interruption 4
// Pin 3 pour interruption 1     ou    Pin 20 pour interruption 3     ou      Pin 19 pour interruption 5

// ******************************* VARIABLES *********************************************
const float pi = 3.14159267;

// Caractéristiques encodeurs et moteur
float gear_ratio= 19;                         // rapport de réduction du réducteur par défaut
int nb_ticks_par_tour=64;                     // nombre de ticks de l'encodeur par défaut

// variables d'interruptions
volatile signed long encoder_D = 0;
volatile signed long encoder_G = 0;
volatile float omega_D = 0;                   //  en rad/s
volatile float omega_G = 0;                   //  en rad/s
long pos_D = 0;                               // variables globales pour les valeurs des encodeurs
long pos_G = 0;
long codeurDeltaPos_D;
long codeurDeltaPos_G;
volatile double dt = 0.01;                    // période d'échantillonnage

int mode;
// ******************************** SETUP *************************************************
void setup() {

// ********* A. Configuration port série  *************
  Serial.begin(115200); // configuration de la vitesse de connexion série : 115200 bauds (ou bits par seconde)

// ********** B. Initialisation des encodeurs et interruptions associées ***********************
  pinMode(encoder_D_PinA, INPUT); 
  digitalWrite(encoder_D_PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder_D_PinB, INPUT); 
  digitalWrite(encoder_D_PinB, HIGH);       
  pinMode(encoder_G_PinA, INPUT); 
  digitalWrite(encoder_G_PinA, HIGH);       
  pinMode(encoder_G_PinB, INPUT); 
  digitalWrite(encoder_G_PinB, HIGH);       

// PinA 2 pour interrupt 0  ou  PinA 21 pour interrupt 2   ou    PinA 18 pour interrupt 4
// PinB 3 pour interrupt 1  ou  PinB 20 pour interrupt 3   ou    PinB 19 pour interrupt 5
  attachInterrupt(4, do_encoder_D_A, CHANGE);  
  attachInterrupt(5, do_encoder_D_B, CHANGE);  
  attachInterrupt(3, do_encoder_G_A, CHANGE);  
  attachInterrupt(2, do_encoder_G_B, CHANGE);  

Serial.println("Entrer le nombre de ticks utilisé par tour sur vos encodeurs (nb position x nb pistes x nb fronts : ");
  while (Serial.available()==0){}                                            // attente de valeur ou depart
  nb_ticks_par_tour = Serial.parseFloat();                                   // nombre complet de ticks (avec pistes et fronts)

Serial.println("Entrer la valeur du rapport de réduction r utilisé (exemple 19) : ");
  while (Serial.available()==0){}                                              // attente de valeur ou depart
  gear_ratio = Serial.parseFloat();

// ************ C. Interruption pour le calcul de vitesse ********************
// lance la fonction isrt (fin de programme) suivant une période choisie
  FlexiTimer2::set(50, 1/1000., isrt);                                                    // résolution timer = 1 ms, période 50 ms
  FlexiTimer2::start();

delay(100);                                                
}
// ******************************************************************************************
// ********************************* PROGRAMME ***********************************************
void loop() {
     // ****************************************************************************************** 
    // ************* 0.Choix du mode de fonctionnement **************************** 
      choix_mode:  
        
        Serial.println("Choisissez votre mode de fonctionnement :..."); 
        Serial.println("1 = Affichage des valeurs brutes des encodeurs");
        Serial.println("2 = Affichage des positions en ° en sortie ");
        Serial.println("3 = Affichage des vitesses en rad/s en sortie ");

        while (Serial.available()==0){}                                              // attente de valeur ou depart
        mode = Serial.parseInt();                                                    

     // ****************************************************************************************** 
     // --------------------------- 1. Affichage des positions encodeurs ------------------------------

     if (mode == 1)
      {
       Serial.println("Faire tourner manuellement vos roues ou codeurs ");
       delay(500); 
       while (1) {
          Serial.print("Codeur_Droit = "); 
          Serial.print(pos_D);
          Serial.print(",");
          Serial.print("  ; Codeur_Gauche = "); 
          Serial.println(pos_G);
          delay(200);
       }                                                                                                    // boucle infinie
      }

     // --------------------------- 2. Affichage des positions angulaires en ° des encodeurs ------------------------------

     if (mode == 2)
      {
       Serial.println("Faire tourner manuellement vos roues ou codeurs ");
       delay(500); 
       while (1) {
          Serial.print("Sortie_Droite = "); 
          Serial.print(pos_D*360/(nb_ticks_par_tour*gear_ratio));
          Serial.print("° ,");
          Serial.print("  ; Sortie_Gauche = "); 
          Serial.print(pos_G*360/(nb_ticks_par_tour*gear_ratio));
          Serial.println("°");
          delay(200);
       }        
      }

     // --------------------------- 3. Affichage des vitesses en rad/s des  encodeurs ------------------------------

     if (mode == 3)
      {
       Serial.println("Faire tourner manuellement ou via une alim. (pour moteur) vos encodeurs ");
       delay(500); 
       while (1) {
          Serial.print("vitesse Codeur_Droit = "); 
          Serial.print(omega_D);
          Serial.print(" rad/s ,");
          Serial.print("  ; vitesse Codeur_Gauche = "); 
          Serial.print(omega_G);
          Serial.println(" rad/s");
          delay(200);
       }                                                                                                    // boucle infinie
      }
    
   delay(20000);
}


// ***************  Procédures d'Interruptions  **************************************
// Procédures d'Interruptions
void isrt(){

  // Nombre d'impulsions codeur pendant période d'échantillonnage
  codeurDeltaPos_D = encoder_D;                                                      // lecture du codeur
  pos_D = pos_D + codeurDeltaPos_D;                                                  // pour un accès à l'extérieur des variables volatile  
  encoder_D = 0;                                                                     // reset codeur

  codeurDeltaPos_G = encoder_G;                                                      // lecture du codeur
  pos_G = pos_G + codeurDeltaPos_G; 
  encoder_G = 0;                                                                     // reset codeur 

  omega_D = float(codeurDeltaPos_D*(2*3.14116)/(gear_ratio*nb_ticks_par_tour*dt));
  omega_G = float(codeurDeltaPos_G*(2*3.14116)/(gear_ratio*nb_ticks_par_tour*dt));

}


// **********************************************************************************
void do_encoder_D_A() {
  if (digitalReadFast2(encoder_D_PinA) == digitalReadFast2(encoder_D_PinB)) 
  {encoder_D++;} 
  else {encoder_D--;}
}

void do_encoder_D_B() {
  if (digitalReadFast2(encoder_D_PinA) == digitalReadFast2(encoder_D_PinB)) 
  {encoder_D--;} 
  else {encoder_D++;}
}  
void do_encoder_G_A() {
  if (digitalReadFast2(encoder_G_PinA) == digitalReadFast2(encoder_G_PinB)) 
  {encoder_G++;} 
  else {encoder_G--;}
}

void do_encoder_G_B() {
  if (digitalReadFast2(encoder_G_PinA) == digitalReadFast2(encoder_G_PinB)) 
  {encoder_G--;} 
  else {encoder_G++;}  
}  

