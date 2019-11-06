
// Lecture des signaux A et B d'un encodeur
//#######################   Bibliothèque   ###############################
#include <digitalWriteFast.h>                                   //bibliothèque de plus bas niveau pour ne pas perdre de donnée sur un codeur

//########################   Variables ######################################
#define encoder_pin_a 18                                        //paramètrage encodeur piste a 
#define encoder_pin_b 19                                        //paramètrage encodeur piste b

//########################   Setup du programme ######################################
void setup() {
Serial.begin(115200);                                           // Ouverture du port série et définition de la vitesse
//********************  paramétrage ports numériques *******************
pinMode(encoder_pin_a,INPUT);                                   // La voie choisie pour le piste a est déclarée comme une entrée...
 digitalWrite(encoder_pin_a,HIGH);                              // ...forcée à l'état haut  
pinMode(encoder_pin_b, INPUT);                                  // La voie choisie pour le piste b est déclarée comme une entrée...
 digitalWrite(encoder_pin_b, HIGH);                             // ...forcée à l'état haut
//**********************paramètrage interruption**************************
}

//########################   programme à effectuer en boucle ######################################
void loop() 
      {
      Serial.print(digitalReadFast2(encoder_pin_a));                                  // Afficher la valeur encoder
      Serial.print(",");
      Serial.println(digitalReadFast2(encoder_pin_b));
      }
  






