// Programme de COMMANDE EN VITESSE D'UN MOTEUR CC :
      // réponse indicielle et fréquentielle (créneau et sinus)
      // Avec réglage de la fréquence du hacheur (optionnel)
      // Pour Shield KA03 ou VMA03 (orange avec mesure de courant)
// CPGE TSI Monge

/* CABLAGE :
moteur D sur A: Black - ou 1     RED power + ou 2         GREEN Hall sensor -    BLUE Hall sensor 3.3-5V     YELLOW encoder A       WHITE  encoder B     
                                                                                               18                     19
Shield KA03 (valeurs modifiables)               Shield VMA03 (mesure courant, orange)                                                                                           20                     21
dir_D :  2        pwm_D :  3                    dir_D :  12        pwm_D :  3  
dir_G :  8        pwm_G :  11                   dir_G :  13        pwm_G :  11

Sur Arduino Mega:
Pins 4 and 13: controlled by timer0         Pins 11 and 12: controlled by timer1          Pins 9 and10: controlled by timer2
Pin 2, 3 and 5: controlled by timer 3       Pin 6, 7 and 8: controlled by timer 4
Attention, le timer 1 modifie aussi les delay !!!
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
byte type_signal;
byte type_asserv;
int shield=1;                                   // 1 pour shield classique  ; 2 pour un shield avec mesure de courant (VMA03)
// ******************************* VARIABLES Moteurs & Encodeurs *********************************************
int potar_value =0;
int commande = 0; 
float rapport_cyclique = 0;

// Caractéristiques encodeurs et moteur
float gear_ratio= 19;                         // rapport de réduction du réducteur par défaut
int nb_ticks_par_tour=64;                     // nombre de ticks de l'encodeur par défaut

// valeurs suivant shield moteur (modifiables)
int dir_D=2;                            //direction control for motor D
int pwm_D=3;                            //PWM control for motor D

// variables d'interruptions
long pos_D = 0;                          // variables globales pour les valeurs des encodeurs
long codeurDeltaPos_D;
volatile signed long encoder_D = 0;
float omega_D = 0;                            //  en rad/s
double dt = 0.01;                             //période d'échantillonnage

// Enregistrement 
int index =0;
const int nb_data=100 ;                       // nombre de données pour l'acquisition (modifiable dans la limite de memoire disponible)
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
    Serial.flush();

// *********** B. Choix du motoréducteur (optionnel)**************
/*
Serial.println();
Serial.println("Entrer le nombre de ticks utilisé par tour sur vos encodeurs (nb position x nb pistes x nb fronts : ");
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
        Serial.println("Réponse Indicielle ou Frequentielle d'un moteur CC (en Boucle Ouverte) ");
        Serial.println("******************************************************************");         
        Serial.println();
        Serial.println("Entrer la tension d'alimentation du moteur (entre 6 et 12) ");
        while (Serial.available()==0){}                                                    // attente de valeur ou depart
        int tension_alim = Serial.parseInt();
        Serial.println();
        Serial.println("Veiller à ce que rien ne gêne la rotation du moteur (seul ou sur systeme)");
        Serial.println(); 
        Serial.println("Entrer 'e' pour lancer l'echelon Indiciel");
        Serial.println("Entrer 'f' pour une reponse Frequentielle ");                      // attente du départ
        while (Serial.available()==0){}                                                    // attente de valeur
        byte ordre =Serial.read();
        
        if (ordre=='e'){                                                            // en cas d'entrée différente (rampe r , sinusoide s ,triangle t...)
     // ********** Echelon de Vitesse ********
                 CommandeMoteur(pwm_D,dir_D, 255);
                 temps_ms[0]= 0 ; 
                 entree[0]= tension_alim ;                                             // A modifier si la tension de commande est différente
                 vitesse_D[0]= 0;
                 start_time = millis();   
              // ****** enregistrement de n valeurs *******
                 for (index=1; index<nb_data; index++){                               
                    temps_ms[index]= millis()- start_time; 
                    entree[index]= tension_alim ;                                             
                    vitesse_D[index]= omega_D;
                    delay(5);                                                        // période 5ms pour acquérir 200 points en 1s
                 }
                 analogWrite(pwm_D, 0);
                 
              // ****** envoi décalé sur port série *******                                                        
                 sendData();
                 delay(1000);                                                       // pendant 1 seconde
           }
     // ********** signal frequentiel ********
              if (ordre=='f'){
                Serial.println("Entrer la frequence en hz (entre 0.5 et 10 Hz):  ");
                while (Serial.available()==0){}                                          // attente de valeur ou depart
                while (Serial.available() > 0) {frequence = Serial.parseFloat();}
                periode_ms = int(1000/frequence);           

                Serial.println("Entrer 'c' pour un signal en créneau ");
                Serial.println("Entrer 's' pour un signal en sinusoïde ");
                while (Serial.available()==0){}                                          // attente de valeur ou depart
                while (Serial.available() > 0) {type_signal = Serial.read();}
                      // ***** Signal créneau *****
                      if (type_signal == 'c'){
                              index=0;
                              temps_ms[0]=0; 
                              start_time = millis(); 
                              int period = 0;                                            // index de période
                              while (temps_ms[index] < record_time){
                                            if (temps_ms[index] > (periode_ms*(period+1))){period++;} ;
                                            entree[index]= tension_alim * pow(-1,temps_ms[index] > ((period+0.5)*periode_ms)) ;
                                            CommandeMoteur(pwm_D,dir_D, entree[index]*255/tension_alim);
                                            delay(10);                                            // delai indispensable
                                            vitesse_D[index]= omega_D;                             //  vitesse en m/s
                                            index++;
                                            delay(int(record_time/nb_data));
                                            temps_ms[index]= millis()- start_time;
                                }
                              analogWrite(pwm_D, 0);
                              sendData();                                            // ****** envoi décalé sur port série *******                                                             
                      }
                      // ***** Signal sinusoidal *****
                        if (type_signal == 's'){
                              temps_ms[0]= 0 ; 
                              index=0; 
                              start_time = millis(); 

                              while (temps_ms[index] < record_time){
                                    entree[index]= tension_alim * sin(deuxPi*frequence*temps_ms[index]/1000) ;
                                    CommandeMoteur(pwm_D,dir_D, entree[index]*255/tension_alim);
                                    delay(10);                                              // delai indispensable                                                                                                    
                                    vitesse_D[index]= omega_D;                             //  vitesse en m/s
                                    index++;
                                    delay(int(record_time/nb_data));
                                    temps_ms[index]= millis()- start_time;
                                }
                                analogWrite(pwm_D, 0);
                                sendData();                     // ****** envoi décalé sur port série *******
                      }
              }
delay(1000);
}

// ***************** Envoi série décalé ***************
void sendData(){
      Serial.println("Format: temps, tension de commande, vitesse");
      for (index=0;index<nb_data;index++){
        float temps = temps_ms[index]/1000.0;                // temps_ms ramené en secondes
        Serial.print(temps);                
        Serial.print(",");
        Serial.print(entree[index]);
        Serial.print(",");
        Serial.println(vitesse_D[index]);
        delay(10);
      }
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
void isrt(){

  // Nombre d'impulsions codeur pendant période d'échantillonnage
  codeurDeltaPos_D = encoder_D;                                                      // lecture du codeur
  pos_D = pos_D + codeurDeltaPos_D;                                                  // pour un accès à l'extérieur des variables volatile  
  encoder_D = 0;                                                                     // reset codeur

  omega_D = float(codeurDeltaPos_D*(2*Pi)/(gear_ratio*nb_ticks_par_tour*dt));
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

