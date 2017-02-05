/* 
Shield Moteurs Ada
Utilisation des PIN analogiques 5 et 6 pour les moteurs DC
Utilisation des PIN numériques 9 et 10 pour les servo moteurs 
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define ENGINE_M1 1
#define ENGINE_M4 4
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *adaRightMotor = AFMS.getMotor(ENGINE_M1);
// You can also make another motor on port M4
Adafruit_DCMotor *adaLeftMotor = AFMS.getMotor(ENGINE_M4);


/* Senseur de distance Ultrason HC-SR04:
 VCC sur Arduino 5v 
 GND sur Arduino GND
 Echo sur Arduino broche 7 
 Trig sur Arduino broche 8
*/
#define PIN_ECHO_SR04 7 // broche Echo 
#define PIN_TRIG_SR04 8 // broche Trigger (declenchement)
#define PIN_LED_13 13 // LED de la carte Arduino (branché sur la broche 13)
 
int maximumRange = 400; // distance Maximale acceptée (en cm)
int minimumRange = 0;   // distance Minimale acceptée (en cm)
long duration, distance; // Durée utilisé pour calculer la distance


/*
 * Interrupteur INPUT_PULLUP + LED
 */
#define PIN_START_BUTTON 6  // Boutton Start 
#define PIN_START_LED 5     // LED allumée si start

/*
 * Etats de la voiture
 */
#define MIN_SPEED_FOR_TURN 75
#define SPEED_FORWARD 120
#define STATE_STOP 0
#define STATE_GO_FORWARD 1
#define STATE_GO_BACKWARD 2
#define STATE_TURN_RIGHT_FORWARD 3
#define STATE_TURN_LEFT_FORWARD 4
#define STATE_TURN_RIGHT_BACKWARD 5
#define STATE_TURN_LEFT_BACKWARD 6

/*
 * Variables globales
 */
boolean bSerialBegin = false;
boolean bStart = false;
unsigned long currentMillis = millis();

class AutonomousCar
{
  // These are initialized at startup
  public:
    Adafruit_DCMotor *myRightMotor;
    Adafruit_DCMotor *myLeftMotor;
    int iCarSpeed;
    int previousState;
    int currentState;
   
  public:
    AutonomousCar();
    
    void initCar();
    void printState();
    void stopCar();
    void turnRightFW();
    void turnLeftFW();
    void goForward();
    void goBackward();
    void setCarSpeed(int iSpeed);
};

AutonomousCar::AutonomousCar()
{
  iCarSpeed = 0;
  previousState = STATE_STOP;
  currentState = STATE_STOP;
}

void AutonomousCar::initCar()
{
  myRightMotor = adaRightMotor;
  myLeftMotor = adaLeftMotor;
  iCarSpeed = 0;

  Serial.println("Init Car");
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myRightMotor->setSpeed(0);
  myLeftMotor->setSpeed(0);

  // turn off motor
  myRightMotor->run(RELEASE);
  myLeftMotor->run(RELEASE);
  Serial.println("RELEASE");

  previousState = STATE_STOP;
  currentState = STATE_STOP;
}

void AutonomousCar::printState(){
  switch (currentState){
    case STATE_STOP :
      Serial.println("STATE_STOP");
      break;
    case STATE_GO_FORWARD:
      Serial.println("STATE_GO_FORWARD");
      break;
    case STATE_GO_BACKWARD:
      Serial.println("STATE_GO_BACKWARD");
      break;
    case STATE_TURN_RIGHT_FORWARD:
      Serial.println("STATE_TURN_RIGHT_FORWARD");
      break;
    case STATE_TURN_LEFT_FORWARD:
      Serial.println("STATE_TURN_LEFT_FORWARD");
      break;
    case STATE_TURN_RIGHT_BACKWARD:
      Serial.println("STATE_TURN_RIGHT_BACKWARD");
      break;
    case STATE_TURN_LEFT_BACKWARD:
      Serial.println("STATE_TURN_LEFT_BACKWARD");
      break;
    default:
      Serial.println("????");
  } // switch
}

void AutonomousCar::setCarSpeed(int iSpeed){
  iCarSpeed = iSpeed;
  myRightMotor->setSpeed(iCarSpeed);
  myLeftMotor->setSpeed(iCarSpeed+10);
  Serial.print("Set Car speed "); Serial.println(iCarSpeed);
}

void AutonomousCar::stopCar(){
  setCarSpeed(0);
  myRightMotor->run(RELEASE);
  myLeftMotor->run(RELEASE);
  previousState = currentState;
  currentState = STATE_STOP;
  Serial.println("Stop Car");
}

void AutonomousCar::goForward(){
  myRightMotor->run(FORWARD);
  myLeftMotor->run(FORWARD);
  previousState = currentState;
  currentState = STATE_GO_FORWARD;
  Serial.println("Go Forward");
}

void AutonomousCar::goBackward(){
  myRightMotor->run(BACKWARD);
  myLeftMotor->run(BACKWARD);
  previousState = currentState;
  currentState = STATE_GO_BACKWARD;
  Serial.println("Go Backward");
}

void AutonomousCar::turnRightFW(){
  setCarSpeed(0);
  goForward();
//  iCarSpeed = 0;
//  myRightMotor->setSpeed(iCarSpeed);
  myLeftMotor->setSpeed(MIN_SPEED_FOR_TURN);
  previousState = currentState;
  currentState = STATE_TURN_RIGHT_FORWARD;
  Serial.println("Turn Right");
}

void AutonomousCar::turnLeftFW(){
  goForward();
  iCarSpeed = 0;
  myRightMotor->setSpeed(MIN_SPEED_FOR_TURN);
  myLeftMotor->setSpeed(iCarSpeed);
  previousState = currentState;
  currentState = STATE_TURN_LEFT_FORWARD;
  Serial.println("Turn Left");
}

/*
 * 
 * Fonction détecteur SR04
 * 
 * Partie du code continuellement exécuté
 * Son but est d'effectuer un cycle de détection pour déterminer 
 * la distance de l'objet le plus proche (par réverbération de 
 * l'onde sur ce dernier)
 *
 */
void testDistanceSR04(){
 // Envoi une impulsion de 10 micro seconde sur la broche "trigger" 
 digitalWrite(PIN_TRIG_SR04, LOW); 
 delayMicroseconds(2); 

 digitalWrite(PIN_TRIG_SR04, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(PIN_TRIG_SR04, LOW);

 // Attend que la broche Echo passe au niveau HAUT 
 // retourne la durée
 duration = pulseIn(PIN_ECHO_SR04, HIGH);
 
 //Calculer la distance (en cm, basé sur la vitesse du son).
 distance = duration/58.2;
 
 // Si la distance mesurée est HORS des valeurs acceptables
 if (distance >= maximumRange || distance <= minimumRange){
    /* Envoyer une valeur négative sur la liaison série.
       Activer la LED pour indiquer que l'erreur */
   Serial.print("Distance invalide : ");Serial.println(distance);
   digitalWrite(PIN_LED_13, HIGH); 
 }
 else {
   /* Envoyer la distance vers l'ordinateur via liaison série.
      Eteindre la LED pour indiquer une lecture correcte. */
   Serial.print("Distance (cm) : ");Serial.println(distance);
   digitalWrite(PIN_LED_13, LOW); 
 }
 
 //Attendre 50ms avant d'effectuer la lecture suivante.
 delay(50);
}


AutonomousCar MyCar;


/*
 * 
 * 
 *    SetUP
 * 
 * 
 */
void setup() {
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
//  OCR0A = 0xAF; // Output Compare Register A = 1010 1111, 175
//  TIMSK0 |= _BV(OCIE0A); // TIMSK0 – Timer/Counter Interrupt Mask Register - enable timer compare interrupt - TIMSK0: 3 _BV(OCIE0A): 2

  Serial.begin(9600);      // sets the serial port to 9600
  Serial.println("Setup");

  AFMS.begin();  // create with the default frequency 1.6KHz
  MyCar.initCar();
  Serial.println("End Init Car");
  MyCar.setCarSpeed(SPEED_FORWARD);
  MyCar.goForward();
    
  // Activer les broches pour le capteur ultra son SR 04
  pinMode(PIN_TRIG_SR04, OUTPUT);
  pinMode(PIN_ECHO_SR04, INPUT);
  pinMode(PIN_LED_13, OUTPUT); // activer la LED sur la carte (si nécessaire)
  Serial.println("End Init PIN SR04");

  //Activer bouton poussoir
  pinMode(PIN_START_BUTTON, INPUT_PULLUP);
  pinMode(PIN_START_LED, OUTPUT);

}


// Interrupt is called once a millisecond, 
/*SIGNAL(TIMER0_COMPA_vect) 
{
} 
*/

#define DISTANCE_SECURITE 20
#define TIME_OUT_TURN_RIGHT 3000
#define TIME_OUT_GO_BACKWARD 2000

unsigned long ulStartTurnRight = 0;
unsigned long ulStartGoBW = 0;

void loop() {
  
  MyCar.printState();
  currentMillis = millis();
//  Serial.print("currentMillis : ");Serial.println(currentMillis);
//  Serial.print("ulStartTurnRight : ");Serial.println(ulStartTurnRight);
//  Serial.print("ulStartGoBW : ");Serial.println(ulStartGoBW);
  testDistanceSR04();

  switch (MyCar.currentState){
    case STATE_GO_FORWARD:
      if (distance < DISTANCE_SECURITE)
      {
        ulStartTurnRight = millis();
        Serial.print("ulStartTurnRight : ");Serial.println(ulStartTurnRight);
        MyCar.turnRightFW();
      }
      break;
    
    case STATE_TURN_RIGHT_FORWARD:
      if (distance < DISTANCE_SECURITE)
      {
        if (currentMillis - ulStartTurnRight > TIME_OUT_TURN_RIGHT)
        {
          Serial.print("currentMillis - ulStartTurnRight : ");Serial.println(currentMillis - ulStartTurnRight);
          MyCar.setCarSpeed(SPEED_FORWARD);
          MyCar.goBackward();
          ulStartGoBW = millis();
          Serial.print("ulStartGoBW : ");Serial.println(ulStartGoBW);
        }
      }
      else
      {
        MyCar.setCarSpeed(SPEED_FORWARD);
        MyCar.goForward();
      }
      break;
    
    case STATE_GO_BACKWARD:
      if (currentMillis - ulStartGoBW > TIME_OUT_GO_BACKWARD)
      {
        Serial.print("currentMillis - ulStartGoBW : ");Serial.println(currentMillis - ulStartGoBW);
        MyCar.setCarSpeed(SPEED_FORWARD);
        MyCar.goForward();
      }
      break;
    
    default:
      MyCar.setCarSpeed(SPEED_FORWARD);
      MyCar.goForward();
    
  } // switch
}

/*
    void stopCar();
    void turnRightFW();
    void turnLeftFW();
    void goForward();
    void goBackward();
    void setCarSpeed(int iSpeed);

#define STATE_STOP 0
#define STATE_GO_FORWARD 1
#define STATE_GO_BACKWARD 2
#define STATE_TURN_RIGHT_FORWARD 3
#define STATE_TURN_LEFT_FORWARD 4
#define STATE_TURN_RIGHT_BACKWARD 5
#define STATE_TURN_LEFT_BACKWARD 6


*/


