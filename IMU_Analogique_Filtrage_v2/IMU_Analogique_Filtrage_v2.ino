// Filtrage passe bas, passe haut et filtre complémentaire de l'accéléromètre et gyroscope analogiques
// Réalisés dans la boucle LOOP

//------------ déclaration des constantes du programmes à fixer au départ --------
float Te = 0.005 ;                                  // période d'échantillonnage
float dt = Te ;                                     // dt = intervalle de temps en secondes pour l'intégration
float tau = 0.159;                                  // valeur de constante de temps pour le filtre passe-bas 
float tauh = tau;                                   // valeur de constante de temps pour le filtre passe-haut

//---------- déclaration pour l'accéléromètre, des variables du programme -----------
int broche_accelero = 0 ;                           // la broche de l'arduino sur laquelle le fil accéléro est branché
float valeur_accelero_horiz = 507 ;                 // valeur accelero à l'horizontale initialisée à 507 mais ajustée plus loin, au départ du programme !
float mesure_accelero = 0 ;                         // variable qui recevra les mesures de l'accéléromètre (initialisée à 0)
float valeur_accelero_filtre = 0 ;                  // variable qui recevra le calcul des valeurs filtrées (initialisée à 0)
float valeur_accelero_filtre_precedente = 0 ;       // variable des valeurs filtrées précédentes  (initialisée à 0)
float gain_accelero=0.89603665  ;                   // le gain de l'accéléromètre en degrés par point)

//---------- déclaration pour le gyromètre, des variables du programme -----------
int broche_gyro = 1 ;                               // la broche de l'arduino sur laquelle le fil gyro est branché
float valeur_gyro_arret = 0 ;                       // valeur gyro à l'arrêt initialisée à zéro mais ajustée plus loin, au départ du programme !
float gain_gyro =0.3255208333333  ;                 // le gain du gyromètre en ((degrés par seconde)) par point) 
float mesure_gyro = 0 ;                             // variable qui recevra les mesures du gyromètre (initialisée à 0)
float ecart_gyro = 0 ;                              // variable pour le calcul de l'écart par rapport à la position arrêtée
float vitesse_angulaire = 0 ;                       // variable pour le calcul de la vitesse angulaire
float valeur_position_degres = 0 ;                  // variable qui recevra le clacul des valeurs de la position (initialisée à 0)
float valeur_position_degres_precedente = 0;        // pour le filtre passe-haut
float valeur_position_degres_FPH = 0 ;              // variable qui recevra le clacul des valeurs de la position après filtrage passe-haut
float valeur_position_degres_FPH_precedente = 0 ;   // pour le filtre passe-haut
float valeur_filtre_complementaire = 0;             // pour le filtre complémentaire


// ---------------------variables pour la gestion du temps ----------------------------------------------------
long mesure_temps_periode = 0 ;                     // déclaration de la variable qui mesure le temps de la période de calcul
long test_periode = 0 ;                             // déclaration de la variable qui va tester si la durée d'une période est écoulée
float temps_sec  = 0;                               // variable de mesure du temps
int i = 1 ;                                         // pour incrémentation sur l'initialisation du gyro


// --------- initialisation des procédures ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200) ;                            // initialisation du port série pour visu des résultats : 
                                                    // attention il faudra régler la valeur dans l'interface Arduino à la même valeur
  mesure_temps_periode = millis() ;                 // initialisation de la variable de mesure de période
  Serial.println(" initialisation capteurs en cours ; balancier horizontal : ne pas faire bouger ") ;  
  while (i <= 100) {
    valeur_gyro_arret = valeur_gyro_arret + analogRead(broche_gyro) ;
    valeur_accelero_horiz = valeur_accelero_horiz + analogRead(broche_accelero) ;       
    i=i+1 ;
    delay(20) ;
  }
  valeur_gyro_arret = valeur_gyro_arret / 100 ;
  valeur_accelero_horiz = valeur_accelero_horiz / 100;
} // fin du setup


// ---------- boucle sans fin du programme -----------------------------------------------------------------------
void loop(){
  test_periode = millis() - mesure_temps_periode ;                                        // ----------> la suite ne s'exécute que si la durée d'une période est écoulée
  if (test_periode >= (dt * 1000)) {                                                      // pour savoir si on a atteint la durée d'une période 
    temps_sec = millis()/1000.0; 
 
    // ---------- réalisation des calculs pour le filtrage de l'accéléromètre ----------------------->
    mesure_accelero =gain_accelero*(analogRead(broche_accelero)- valeur_accelero_horiz) ;  // mesure_accelero : valeur lue sur 10 bits (0 à 1023 pour 0 à 5V)
    // ----> ci-dessous on place l'équation du filtre passe bas :  
    valeur_accelero_filtre = (Te/(tau+Te)) * mesure_accelero + (tau/(tau+Te)) * valeur_accelero_filtre_precedente ;      
    valeur_accelero_filtre_precedente = valeur_accelero_filtre ;                          // pour préparer le calcul suivant
    // <-----------------------fin des calculs pour le filtrage accéléromètre --------------------------------

    // ---------- réalisation des calculs pour la mesure de position par le gyromètre ----------------------->
    mesure_gyro = analogRead(broche_gyro) ;                                               // valeur lue sur 10 bits (0 à 1023 pour 0 à 5V)
    ecart_gyro = mesure_gyro - valeur_gyro_arret ;                                        // calcul de la variation par rapport à la valeur à l'arrêt
    vitesse_angulaire = ecart_gyro * gain_gyro ;                                          // calcul de la vitesse angulaire en degrés par seconde 
    // ----> ci-dessous on place l'équation de l'intégration :
    valeur_position_degres = valeur_position_degres + vitesse_angulaire * dt ;            // ici on a l'intégration pour le calcul de la position !!
    // ----> ci-dessous on place l'équation du filtrage passe-haut :    
    valeur_position_degres_FPH = (valeur_position_degres - valeur_position_degres_precedente + valeur_position_degres_FPH_precedente)*tauh/(tauh + dt);
    valeur_position_degres_precedente = valeur_position_degres;
    valeur_position_degres_FPH_precedente = valeur_position_degres_FPH;
    // <-----------------------fin des calculs pour la mesure de position par le gyromètre -----------------
 
    valeur_filtre_complementaire = 0.5*valeur_accelero_filtre + 0.5 * valeur_position_degres_FPH;  // réalisation du filtre complémentaire 
       
  //---------------------------------  "génération des messages à afficher"-------------------------------------------------

    Serial.print("t : ") ;   
    Serial.print(temps_sec);                                                  // envoi de la valeur "temps" pour affichage
    Serial.print("  accelero filtre : ") ;    
    Serial.print(valeur_accelero_filtre) ;                                    // envoi de la valeur filtrée , pour affichage    
  //    Serial.print(" ; gyro : ") ;
  //    Serial.print( vitesse_angulaire );                                    // envoi de la valeur "vitesse" pour affichage
  //    Serial.print(" ; position : ") ;
  //    Serial.print(valeur_position_degres) ;                                // envoi de la valeur de position calculée, pour affichage
    Serial.print("  gyro passe haut : ") ;
    Serial.print(valeur_position_degres_FPH) ;                                // envoi de la valeur de position calculée, pour affichage
    Serial.print("  filtre complementaire : ") ;    
    Serial.print(valeur_filtre_complementaire) ;                              // envoi de la valeur filtrée , pour affichage        
    Serial.println() ;
 
  // fin de génération des messages à afficher
   // Serial.println(test_periode);                                           // pour vérification du temps de calcul
    test_periode = 0 ;                                                        // réinitialisation de la variable de test de la durée de période
    mesure_temps_periode = millis() ;                                         // pour permettre le calcul de la durée de la période suivante   
  }                                                                           // fin du if
}                                                                             // fin de la boucle loop




  
