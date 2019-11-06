// Programme d'Asservissement d'un moteur CC
  // reglge PID
// CPGE TSI Monge

/* CABLAGE : 
moteur D sur A: Black - ou 1     RED power + ou 2         GREEN Hall sensor -    BLUE Hall sensor 3.3-5V     YELLOW encoder A       WHITE  encoder B     
                                                                                               18                     19
Shield KA03 (valeurs modifiables)               Shield VMA03 (mesure courant, orange)                                                                                           20                     21
dir_D :  2        pwm_D :  3                    dir_D :  12        pwm_D :  3  
dir_G :  8        pwm_G :  11                   dir_G :  13        pwm_G :  11
*/  
//****************************************************************************************
// ***************************** BIBLIOTHEQUES *****************************************
#include <FlexiTimer2.h>
#include <digitalWriteFast.h>              // bibliothèque lecture rapide entrées digitales

// *****************************  PARAMETRAGE *******************************************
#define encoder_D_PinA  18        // adresse des encodeurs
#define encoder_D_PinB  19

// Pin 2 pour interruption 0     ou    Pin 21 pour interruption 2     ou      Pin 18 pour interruption 4
// Pin 3 pour interruption 1     ou    Pin 20 pour interruption 3     ou      Pin 19 pour interruption 5

// ******************************* VARIABLES *********************************************
#define Pi 3.14159267
#define deuxPi 6.283185

int ordre=0 ;
int mode;

byte type_asserv;

// ******************************* VARIABLES Moteurs & Encodeurs *********************************************
int commande = 0; 
float rapport_cyclique = 0;

// Caractéristiques encodeurs et moteur
float gear_ratio= 19;                         // rapport de réduction du réducteur par défaut
int nb_ticks_par_tour=64;                     // nombre de ticks de l'encodeur par défaut

// valeurs suivant shield moteur (confer setup)
int dir_D=2;                                //direction control for motor D
int pwm_D=3;                                //PWM control for motor D

// PID
float kp=0.1;
float ki=0.;
float kd=0.;
float terme_P = 0.;
float terme_I = 0.;
float terme_D = 0.;
int max_I = 50;                              // valeur max du terme intégral


int consigne_D= 1000;                        // consigne en pas codeur par défaut
long pos_D = 0;                             // variables globales pour les valeurs des encodeurs
long erreur_D ;                             // pour l'asservissement
long erreur_prec_D=0;
long delta_erreur_D;
long codeurDeltaPos_D;
volatile double commande_D = 0.;

// variables d'interruptions
volatile signed long encoder_D = 0;
float omega_D = 0;                            //  en rad/s
double dt = 0.01;                             //période d'échantillonnage

// Enregistrement 
int index =0;
const int nb_data=100 ;                       // nombre de données pour l'acquisition
int record_time = 4000;                       // temps_ms d'acquisition en millisecondes
long entree[nb_data];                         //données transmises
float vitesse_D[nb_data];                     //  en rad/s
float temps_ms[nb_data];                         // temps_ms réel en ms

unsigned long current_time = 0;
unsigned long start_time = 0;
float frequence = 0;                  // frequence commande sinusoidale
int periode_ms = 0 ;
// ******************************** SETUP *************************************************
void setup() {

// ********* A. Configuration port série  *************
    Serial.begin(115200); // configuration de la vitesse de connexion série : 115200 bauds (ou bits par seconde)
                 
// *********** B. Choix du motoréducteur **************
/*
Serial.println();
Serial.println("Entrer le nombre de ticks utilisé par tour sur votre encodeur (nb position x nb pistes x nb fronts : ");
  while (Serial.available()==0){}                                            // attente de valeur ou depart
  nb_ticks_par_tour = Serial.parseFloat();                                   // nombre complet de ticks (avec pistes et fronts)
Serial.println(); 
Serial.println("Entrer la valeur du rapport de réduction r utilisé (exemple 19) : ");
  while (Serial.available()==0){}                                              // attente de valeur ou depart
  gear_ratio = Serial.parseFloat();
*/

// ********* C. Configuration frequence PWM *************
/*
  Serial.println(); 
  Serial.println(" Taper 1 si vous souhaitez modifier la fréquence du hacheur: ");
  Serial.println(" Taper 0 sinon ");
  while (Serial.available()==0){}                                                    // attente de valeur ou depart
  ordre =Serial.parseInt();
      if (ordre == 1){         
  Serial.println(" Taper : 1 pour 1kHz (par defaut), 2 pour 4khz , 3 pour 31.4 khz");
  while (Serial.available()==0){}                                                    // attente de valeur ou depart
  int pwm = Serial.parseInt();
  if (pwm == 1)
          {TCCR3B = TCCR3B & 0b11111000 | 0x01; TCCR1B = TCCR1B & B11111000 | 0x01;}
  if (pwm == 2)
          {TCCR3B = TCCR3B & 0b11111000 | 0x02; TCCR1B = TCCR1B & B11111000 | 0x02;}
  if (pwm == 3)
          {TCCR3B = TCCR3B & 0b11111000 | 0x03; TCCR1B = TCCR1B & B11111000 | 0x03;}
      }    
*/
// ****************** D. Configuration des ports ********************************************  
    pinMode(pwm_D, OUTPUT);                // pour le rapport cyclique WM
    pinMode(dir_D, OUTPUT);                // pour le sens de rotation
    
    digitalWrite(dir_D, LOW); 
    analogWrite(pwm_D, 0);  

 // ********** E. Initialisation des encodeurs et interruptions associées ***********************
    pinMode(encoder_D_PinA, INPUT); 
    digitalWrite(encoder_D_PinA, HIGH);       // turn on pullup resistor
    pinMode(encoder_D_PinB, INPUT); 
    digitalWrite(encoder_D_PinB, HIGH);       

// PinA 2 pour interrupt 0  ou  PinA 21 pour interrupt 2   ou    PinA 18 pour interrupt 4
// PinB 3 pour interrupt 1  ou  PinB 20 pour interrupt 3   ou    PinB 19 pour interrupt 5
    attachInterrupt(4, do_encoder_D_A, CHANGE);  
    attachInterrupt(5, do_encoder_D_B, CHANGE);  

// ************ F. Interruption pour le calcul de vitesse ou asservissement ********************
    FlexiTimer2::set(10, 1/1000., isrt);                                                    // résolution timer = 1 ms, période 10 ms
    FlexiTimer2::start();

delay(100);                                                
}
// ******************************************************************************************
// ********************************* PROGRAMME ***********************************************
void loop() {
          Serial.println("Veiller à ce que rien ne gêne le moteur (seul ou sur système)");
          Serial.println("Entrer: 'p' pour un asservissement en position ou 'v' pour un asservissement en vitesse ");

          while (Serial.available()==0){}                                                 // attente de valeur ou depart
          while (Serial.available() > 0) {type_asserv = Serial.read();}

    debut:      
          /******* 1. Choix des parametres du PID ********/
          if (type_asserv == 'p')                                                         // asservissement en position
               {
                Serial.println();
                Serial.println("Entrer: ");
                Serial.println("consigne de position en pas codeur ;  KP ; KI ; KD");        
                Serial.println("exemple :   1500;1;2;0   pour une consigne de position de 1500 pulses avec KP= 1 et KD=2"); 
                 }                                  
 
          else if (type_asserv == 'v')
                {
                Serial.println();
                Serial.println("Entrer: ");
                Serial.println("consigne de vitesse en rad/s ;  KP ; KI ; KD");        
                Serial.println("exemple :   100;1;2;0   pour une consigne de vitesse de 100 rad/s avec KP= 1 et KD=2");         
                 }  
     
          while (Serial.available()==0){}                                                 // attente de valeur ou depart
          while (Serial.available() > 0) {
                consigne_D = Serial.parseFloat();
                kp = Serial.parseFloat()/10;                                              // coefficients pour utiliser des valeurs > 1  plus pratiques                                       
                ki = Serial.parseFloat()/100; 
                kd = Serial.parseFloat()10; 
               }

          reset_Encodeur();                                                               // fonction de reset
          Serial.println("Attendre la fin de commande du moteur... "); 
          
          
          /******* 2. Régulation en boucle PID ********/
          //La position pos et la vitesse omega sont calculées automatiquement dans l'interruption
          for (index=0;index<300;index++) 
                    {
                     // Ecart entre la consigne et la mesure
                    if (type_asserv == 'p')
                        {erreur_D = consigne_D - pos_D;}                                  // asservissement en position
                    else if (type_asserv == 'v')
                        {erreur_D = consigne_D - omega_D;}                                // asservissement en vitesse
                    
                    delta_erreur_D = erreur_D - erreur_prec_D;
                    terme_P= kp * erreur_D;
                    terme_I = terme_I + ki * erreur_D;
                    terme_D = kd * delta_erreur_D;
                    //if (terme_I > max_I) {terme_I=max_I;}                                 // saturation du terme intégral
                    //if (terme_I < -max_I) {terme_I=-max_I;}   
                    commande_D = terme_P + terme_I + terme_D;           

                    // Affichages pour le Debug
                    /*
                    Serial.print(terme_P);                                             
                    Serial.print(";");
                    Serial.print(terme_I);                                              
                    Serial.print(";");
                    Serial.print(terme_D);                                            
                    Serial.print(";");
                    Serial.print(commande_D);                                       
                    Serial.print(";");
                    Serial.print(consigne_D);                                       
                    Serial.print(";");
                    */
                    erreur_prec_D = erreur_D ;
                                       
                    CommandeMoteur(pwm_D,dir_D,int(commande_D));                          // commande PID calculées dans l'interruption
                    Serial.println(consigne_D - erreur_D);
                    delay (10);
              }

           analogWrite(pwm_D, 0);                                                         // arrêt des moteurs
           goto debut;
}
// Fin du programme principal


//  **************** Fonctions utiles *****************
// ***************** Reset de l'encodeur ***************
void reset_Encodeur(){
        pos_D = 0;                                                   
        encoder_D = 0;
        erreur_prec_D=0;
        delta_erreur_D=0;
        codeurDeltaPos_D = 0; 
}
// ***************************
void CommandeMoteur(int pwm_motor_pin,int dir_motor_pin,int pwm_value)
{
    // Saturation de la commande sur 8 bits
    if (pwm_value>255) {pwm_value = 255;}
    if (pwm_value<-255) {pwm_value = -255;}
 
    // Commande du sens de rotation
    if (pwm_value>=0) {
        digitalWrite(dir_motor_pin, LOW);
        analogWrite(pwm_motor_pin, pwm_value);
    }
    if (pwm_value<0) {
        digitalWrite(dir_motor_pin, HIGH);
        analogWrite(pwm_motor_pin, -pwm_value);
    }
}

// ***********************************************************************************
// ***************  Procédures d'Interruptions  **************************************
// Procédures d'Interruptions
//La position pos et la vitesse omega sont calculées automatiquement dans l'interruption
void isrt(){
  // Nombre d'impulsions codeur pendant période d'échantillonnage
  codeurDeltaPos_D = encoder_D;                                                      // lecture du codeur
  pos_D = pos_D + codeurDeltaPos_D;                                                  // pour un accès à l'extérieur des variables volatile  
  encoder_D = 0;                                                                     // reset codeur

  omega_D = float(codeurDeltaPos_D*(2*Pi)/(gear_ratio*nb_ticks_par_tour*dt));        // en rad/s
}


// ***************************
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

