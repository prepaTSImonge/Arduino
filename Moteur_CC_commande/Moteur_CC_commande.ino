// Programme de COMMANDE EN VITESSE D'UN MOTEUR CC :
      // Par potentiomètre ou clavier ou mode auto avec mesure de vitesse 
// CPGE TSI Monge

/* CABLAGE :
moteur sur A: Black - ou 1     RED power + ou 2         GREEN Hall sensor -    BLUE Hall sensor 3.3-5V     YELLOW encoder A       WHITE  encoder B     
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
int shield=1;                               // 1 pour shield classique ; 2 pour shield VMA03 (mesure de courant)
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

// Shield VMA03 
const int brake_D = 9;                               //brake for motor D (non utilisé); 8 pour l'autre
const int current_D_pin = A0;                         // mesure de courant pour moteur Droit sur A0 (A1 pour l'autre)
float coef_current = 0.00295928;                    // resolution sur 10 bits de mesure de courant (5/1024)*(2A/3.3V)

// Potentiomètre
const int potar_pin= A2;                  // potentiomètre sur entrée analogique A2


// variables d'interruptions
long pos_D = 0;                          // variables globales pour les valeurs des encodeurs
long codeurDeltaPos_D;
volatile signed long encoder_D = 0;

float omega_D = 0;                            //  en rad/s
float current_D = 0;                          //  en A
double dt = 0.01;                             //période d'échantillonnage

// ******************************** SETUP *************************************************
void setup() {

// ********* A. Configuration port série  *************
    Serial.begin(115200); // configuration de la vitesse de connexion série : 115200 bauds (ou bits par seconde)
    Serial.flush();

// *********** B. Choix du motoréducteur (optionnel)**************
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

// ********* C. Configuration frequence PWM (optionnel)*************
/*
// 
  
  Serial.println(" Taper 1 si vous souhaitez modifier la fréquence du hacheur: ");
  Serial.println(" Taper 0 sinon ");
  Serial.println("Attention la modification du timer 1 modifie les temps de type delay !!"); 
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

    pinMode(8, OUTPUT);
    digitalWrite(8, HIGH);                  // Permet d'alimenter en 5 V le potentiometre via sortie digitale
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
        Serial.println("Commande d'un moteur par potentiomètre, clavier ou mode auto");
        Serial.println("********************************************************************** ");                                  
        Serial.println();
        Serial.println("Important : Veiller à ce que le moteur tourne librement ");
        Serial.println("Veiller a ce que les fils ne frottent pas ");
        Serial.println();
        Serial.println("Entrer 1 pour un reglage par Potentiometre"); 
        Serial.println("Entrer 2 pour un reglage par Clavier ");
        Serial.println("Entrer 3 pour une variation Automatique ");
        
        while (Serial.available()==0){}                                                        // attente de valeur ou depart
        while (Serial.available() > 0)                                                         // tant que quelquechose est en attente sur le port série
          {ordre = Serial.parseFloat();}
        //*********************************  
        if (ordre == 1){                                                    // ******* Version 1: Commande par potentiomètre ********

                while (1) 
              {
                potar_value = map(analogRead(potar_pin), 0, 1023, 0, 255);
                digitalWrite(dir_D, LOW); 
                analogWrite(pwm_D, potar_value);                              // pwm sur 8 bits
                                                 
 
                Serial.print("potar : ");
                Serial.print(potar_value);
                rapport_cyclique = potar_value/255.0;                         // attention à mettre .0 ou float(potar_value) pour obtenir une valeur réelle
                Serial.print("   rapport cyclique: ");
                Serial.print(rapport_cyclique);
                Serial.print("  Vitesse :  "); 
                Serial.print(omega_D);
                Serial.println("  rad/s  ; ");

              } 
        }
        if (ordre == 2){                                                  // ******* Version 2 : Introduction au clavier de la valeur PWM sur 8 bits ********
                choix_vitesse:
                Serial.println();
                Serial.println("Entrer la commande moteur entre -255 et 255");
                Serial.println("Valeur modifiable après quelques secondes");
                while (Serial.available()==0){}                                                   // attente de valeur
                while (Serial.available() > 0)                                                    // tant que quelquechose est en attente sur le port série
                    {commande=Serial.parseInt();}
   
                CommandeMoteur(pwm_D,dir_D, commande);                                            // fonction de commande du moteur définie en fin de programme

                for (int index=0;index<50;index++)
                    {
                    Serial.print("Vitesse angulaire :  "); 
                    Serial.print(omega_D);
                    Serial.println("  rad/s  ; ");
                    delay(150);   
                    }

                analogWrite(pwm_D, 0);                                                            // Arrêt des moteurs
        
                goto choix_vitesse;
                }
        if (ordre == 3){                                                  // ******* Version 2 : Introduction au clavier de la valeur PWM sur 8 bits ********
                Serial.println();
                Serial.println("Variation automatique");

                int value=0;
                while(value<255)   
                    {
                    CommandeMoteur(pwm_D,dir_D, value);                                            // fonction de commande du moteur définie en fin de programme
                    Serial.print("PWM= ");
                    Serial.println(value);
                    delay(20);
                    value++;   
                    }
                while(value>-255)   
                    {
                    CommandeMoteur(pwm_D,dir_D, value);                                            // fonction de commande du moteur définie en fin de programme
                    Serial.print("PWM= ");
                    Serial.println(value);
                    delay(20);
                    value--;   
                    }
                while(value<0)   
                    {
                    CommandeMoteur(pwm_D,dir_D, value);                                            // fonction de commande du moteur définie en fin de programme
                    Serial.print("PWM= ");
                    Serial.print(value);
                    delay(20);
                    value++;   
                    }                    
                goto choix_vitesse;
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

  if (shield ==2){
  current_D= analogRead(current_D_pin)*coef_current;                                // mesure des courants sur shield Rev3
  }
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

