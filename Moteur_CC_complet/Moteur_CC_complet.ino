// Programme de COMMANDE EN VITESSE D'UN MOTEUR CC :
      // Par potentiomètre ou clavier avec mesure de vitesse 
      // Avec réglage de la fréquence du hacheur
      // Avec mesure de courant(sur shield VMA03)
      // Avec freinage (sur shield VMA03)
      // Pour Shield KA03 ou VMA03 (orange avec mesure de courant)
// CPGE TSI Monge

/* CABLAGE : 
moteur D sur A: Black - ou 1     RED power + ou 2         GREEN Hall sensor -    BLUE Hall sensor 3.3-5V     YELLOW encoder A       WHITE  encoder B     

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
byte type_signal;
byte type_asserv;
int shield;
// ******************************* VARIABLES Moteurs & Encodeurs *********************************************
int potar_value =0;
int commande = 0; 
float rapport_cyclique = 0;

// Caractéristiques encodeurs et moteur
float gear_ratio= 19;                         // rapport de réduction du réducteur par défaut
int nb_ticks_par_tour=64;                     // nombre de ticks de l'encodeur par défaut

// valeurs suivant shield moteur (confer setup)
int dir_D;                            //direction control for motor D
int pwm_D;                            //PWM control for motor D


// Shield VMA03 
const int brake_D = 9;                               //brake for motor D (non utilisé)
const int current_D_pin = A0;                         // mesure de courant pour moteur Droit sur A0   
float coef_current = 0.00295928;                    // resolution sur 10 bits de mesure de courant (5/1024)*(2A/3.3V)

// Potentiomètre
const int potar_pin= A2;                  // potentiomètre sur entrée analogique A2

// PID
float kp=0.1;
float ki=0.;
float kd=0.;
float terme_P = 0.;
float terme_I = 0.;
float terme_D = 0.;
int max_I = 50;                              // valeur max du terme intégral


int consigne_D= 1000;                        // consigne en pas codeur par défaut
long pos_D = 0;                          // variables globales pour les valeurs des encodeurs
long erreur_D ;                           // pour l'asservissement
long erreur_prec_D=0;
long delta_erreur_D;
long codeurDeltaPos_D;
volatile double commande_D = 0.;

// variables d'interruptions
volatile signed long encoder_D = 0;
float omega_D = 0;                            //  en rad/s
float current_D = 0;                          //  en A
double dt = 0.01;                             //période d'échantillonnage

// Enregistrement 
int index =0;
const int nb_data=100 ;                       // nombre de données pour l'acquisition

int record_time = 4000;                       // temps_ms d'acquisition en millisecondes
long entree[nb_data];                         // données transmises
float vitesse_D[nb_data];                     // en rad/s
float courant_D[nb_data];                     // enregistrement de courant
float temps_ms[nb_data];                      // temps_ms réel en ms

const int nb_data2=100 ;                       // nombre de données pour l'acquisition en roue libre
long entree_roue_libre[nb_data2];              // données transmises
float vitesse_roue_libre[nb_data2];            // en rad/s
float courant_roue_libre[nb_data2];            // enregistrement de courant en roue libre
float temps_ms_roue_libre[nb_data2];           // temps_ms réel en ms

unsigned long current_time = 0;
unsigned long start_time = 0;
float frequence = 0;                  // frequence commande sinusoidale
int periode_ms = 0 ;
// ******************************** SETUP *************************************************
void setup() {

// ********* A. Configuration port série  *************
    Serial.begin(115200); // configuration de la vitesse de connexion série : 115200 bauds (ou bits par seconde)
    Serial.flush();

// *********** B1. Choix du shield moteur **************

  Serial.println(" Choisissez votre shield moteur: ");
  Serial.println(" Taper 1 pour shied KA03 (classique) ");
  Serial.println(" Taper 2 pour shield VMA03(orange avec mesure courant) ");                             
  while (Serial.available()==0){}                                                // attente de valeur ou depart
  shield = Serial.parseInt();                                                    // ou parseInt()
      if (shield == 1)
          {dir_D = 2;   pwm_D = 3;}                  // Modifiable suivant jumpers: attention port 9 incompatible avec Timer 2
      if (shield == 2)
          {dir_D = 12;  pwm_D = 3;}                     
// *********** B2. Choix du motoréducteur **************
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
    pinMode(brake_D, OUTPUT);              // pour le frein
    
    digitalWrite(dir_D, LOW); 
    analogWrite(pwm_D, 0);
    digitalWrite(brake_D,LOW);              //desengage le frein  

    pinMode(8, OUTPUT);
    digitalWrite(8, HIGH);              // Permet d'alimenter en 5 V le potentiometre via sorite digitale
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
     // ****************************************************************************************** 
    // ************* 0.Choix du mode de fonctionnement **************************** 
      choix_mode:  
        Serial.println(); 
        Serial.println("Choisissez votre mode de fonctionnement :..."); 
        Serial.println("1 = Affichage de la position brute de l'encodeur et angulaire en sortie ");
        Serial.println("2 = Commande en vitesse du moteur par clavier ou potentiomètre ou auto "); 
        Serial.println("3 = Réponse Indicielle ou Frequentielle du moteur ");        
        Serial.println("4 = Asservissement en Vitesse ou en Position du moteur (PID reglable)");
        Serial.println(" ********************************************");
                             
        while (Serial.available()==0){}                                                    // attente de valeur ou depart
        mode = Serial.parseInt();                                                    // ou parseInt()

     // ****************************************************************************************** 
     // --------------------------- 1. Affichage des positions encodeurs ------------------------------
     if (mode == 1)
      {
       Serial.println("Faire tourner manuellement votre roue ou encodeur ");
       delay(500); 
       while (1) {
          Serial.print("Codeur_D = "); 
          Serial.print(pos_D);
          Serial.print("   vitesse_D = ");
          Serial.print(pos_D*360/(nb_ticks_par_tour*gear_ratio));
          Serial.println("°  ; ");
          delay(500);
       }                                                                                                    // boucle infinie
      }

     // ****************************************************************************************** 
     // --------------------------- 2. Commande en vitesse des moteurs ------------------------------
if (mode == 2)
        {                                              
        Serial.println();
        Serial.println("Important : Veiller à ce que le moteur tourne librement ");
        Serial.println("Veiller a ce que les fils ne frottent pas ");
        Serial.println();
        Serial.println("Entrer 1 pour un reglage par potentiometre"); 
        Serial.println("Entrer 2 pour un reglage par clavier ");
        Serial.println("Entrer 3 pour une variation automatique ");
        
        while (Serial.available()==0){}                                                        // attente de valeur ou depart
        while (Serial.available() > 0)                                                         // tant que quelquechose est en attente sur le port série
          {ordre = Serial.parseFloat();}
        //*********************************  
        if (ordre == 1){                                                    // ******* Version 1: Commande par potentiomètre ********
                while (1) 
              {
                potar_value = map(analogRead(potar_pin), 0, 1023, 0, 255);
                delay(100);
                digitalWrite(dir_D, LOW);                                     // direction moteur   (ou LOW)
                analogWrite(pwm_D, potar_value);                              // pwm sur 8 bits
                
                Serial.print("potar : ");
                Serial.print(potar_value);
                rapport_cyclique = potar_value/255.0;                         // attention à mettre .0 ou float(potar_value) pour obtenir une valeur réelle
                Serial.print("   rapport cyclique: ");
                Serial.print(rapport_cyclique);
                Serial.print("  Vitesse:  "); 
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
                    Serial.print("Vitesse angulaire:  "); 
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
     // ****************************************************************************************** 
     // --------------------------- 3. Identification du moteur ------------------------------
     if (mode == 3)
      {  
          mode3:
          Serial.println();
          Serial.println("Entrer la tension d'alimentation du moteur (entre 6 et 12) ");
          while (Serial.available()==0){}                                                    // attente de valeur ou depart
          int tension_alim = Serial.parseInt();
          Serial.println();
          Serial.println("Veiller à ce que rien ne gêne la rotation du moteur (seul ou sur systeme)"); 
          Serial.println("Entrer 'e' pour lancer l'echelon Indiciel");
          Serial.println("Entrer 'f' pour une reponse Frequentielle ");                      // attente du départ
          while (Serial.available()==0){}                                                    // attente de valeur
          byte ordre =Serial.read();
          
          if (ordre=='e'){                                                            // en cas d'entrée différente (rampe r , sinusoide s ,triangle t...)
              // ******* Commande Moteur *******
     // ********** Echelon de Vitesse ********

                 CommandeMoteur(pwm_D,dir_D, 255);
                 temps_ms[0]= 0 ; 
                 entree[0]= tension_alim ;                                             // A modifier si la tension de commande est différente
                 vitesse_D[0]= 0;
                 start_time = millis();   
              // ****** enregistrement de n valeurs *******
                 for (index=1; index<nb_data; index++){                               
                    temps_ms[index]= (millis()- start_time); 
                    entree[index]= tension_alim ;                                             
                    vitesse_D[index]= omega_D;
                    if (shield==2)
                      {courant_D[index]= current_D;}
                    delay(5);                                                        // période 5ms pour acquérir 200 points en 1s
                 }
                 CommandeMoteur(pwm_D,dir_D, 0);                 

                 // enregistrement supplémentaire des données en roue libre
                 if (shield==2)
                      {for (index=1; index<nb_data2; index++)
                        {
                          temps_ms_roue_libre[index]= (millis()- start_time); 
                          entree_roue_libre[index]= 0 ;
                          courant_roue_libre[index]= current_D;
                          vitesse_roue_libre[index]= omega_D;
                          delay(15);
                          }
                      }
            
              // ****** envoi décalé sur port série *******                                                        
                 sendData();
                 if (shield==2)
                      {sendData2();}                                                       // envoi roue libre
                 
                 delay(1000);                                                       // pendant 1 seconde
                 goto choix_mode;
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
                      if (type_signal == 'c'){
                              index=0;
                              temps_ms[0]=0; 
                              start_time = millis(); 
                              int period = 0;                                            // index de période
                              while (temps_ms[index] < record_time){
                                  if (temps_ms[index] > (periode_ms*(period+1))){period++;} ;
                                  entree[index]= tension_alim * pow(-1,temps_ms[index] > ((period+0.5)*periode_ms)) ;
                                  CommandeMoteur(pwm_D,dir_D, entree[index]*255/tension_alim);
                                                                                                     
                                  vitesse_D[index]= omega_D;                             //  vitesse en m/s
                                  index++;
                                  
                                  delay(int(record_time/nb_data));
                                  temps_ms[index]= (millis()- start_time);
                                }

                              analogWrite(pwm_D, 0);                                           
                                                                          
                      sendData();                     // ****** envoi décalé sur port série *******
                      }

                      if (type_signal == 's'){
                              temps_ms[0]= 0 ; 
                              index=0; 
                              start_time = millis(); 

                              while (temps_ms[index] < record_time){
                                    entree[index]= tension_alim * sin(deuxPi*frequence*temps_ms[index]/1000) ;
                                    CommandeMoteur(pwm_D,dir_D, entree[index]*255/tension_alim);
                                                                                                      
                                    vitesse_D[index]= omega_D;                             //  vitesse en m/s
                                    index++;
                                    delay(int(record_time/nb_data));
                                    temps_ms[index]= (millis()- start_time);
                                }
                                analogWrite(pwm_D, 0);                                           
                                                                          
                                sendData();                     // ****** envoi décalé sur port série *******
                      }
               
              goto choix_mode;
              }
         }

    // ****************************************************************************************** 
     // --------------------------- 4. Asservissement en position ou en vitesse  ------------------------------
        if (mode == 4){
mode4: 
          Serial.println("Veiller à ce que rien ne gêne le moteur (seul ou sur système) ");
          Serial.println("Entrer: 'p' pour un asservissement en position ou 'v' pour un asservissement en vitesse ");

          while (Serial.available()==0){}                                          // attente de valeur ou depart
          while (Serial.available() > 0) {type_asserv = Serial.read();}

 
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
          
          while (Serial.available()==0){}                                          // attente de valeur ou depart
          while (Serial.available() > 0) {
                consigne_D = Serial.parseFloat();
                kp = Serial.parseFloat()/10;               
                ki = Serial.parseFloat()/100; 
                kd = Serial.parseFloat()/10; 
               }

          reset_Encodeur();                                                                   // fonction de reset
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
                    Serial.println(consigne_D - erreur_D);
                    
                    erreur_prec_D = erreur_D ;
                                       
                    CommandeMoteur(pwm_D,dir_D,int(commande_D));                          // commande PID calculées dans l'interruption
       
                    delay (10);
              }
           analogWrite(pwm_D, 0);                                                         // arrêt des moteurs
           goto mode4;
        }
   delay(1000);
}

// ***************** Envoi série décalé ***************
void sendData(){
      Serial.println("Format: temps, tension de commande, vitesse moteur, (optionnel)courant moteur");
      for (index=0;index<(nb_data);index++){
        float temps = temps_ms[index]/1000.0;                // temps_ms ramené en secondes
        Serial.print(temps);                
        Serial.print(",");
        Serial.print(entree[index]);
        Serial.print(",");
        Serial.print(vitesse_D[index]);
        if (shield==2)                                        // pour shield VMA03
            {
             Serial.print(",");
             Serial.print(courant_D[index]);
            }
        Serial.println();
        delay(10);
      }
}

void sendData2(){                                            // pour la roue libre
      Serial.println("Format: temps, tension de commande, vitesse moteur, (optionnel)courant moteur");
      for (index=0;index<(nb_data2);index++){
        float temps = temps_ms_roue_libre[index]/1000.0;                // temps_ms ramené en secondes
        Serial.print(temps);                
        Serial.print(",");
        Serial.print(entree_roue_libre[index]);
        Serial.print(",");
        Serial.print(vitesse_roue_libre[index]);
        Serial.print(",");
        Serial.print(courant_roue_libre[index]);
        Serial.println();
        delay(10);
      }
}


// ***************** Reset des encodeurs ***************
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
    // Changement de sens pour le moteur gauche
    if (dir_motor_pin == 'dir_G'){commande=-commande;}
    
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

  if (shield ==2)
  {current_D= analogRead(current_D_pin)*coef_current;}                               // mesure des courants sur shield Rev3
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
