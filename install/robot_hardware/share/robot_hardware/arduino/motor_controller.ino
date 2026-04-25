/*
 * =============================================================================
 * Motor Controller for Real Robot Navigation (Nav2 Compatible)
 * =============================================================================
 * 
 * Based on the original mapping controller with these additions:
 *   - Cumulative encoder tick output for ROS2 odometry ("E:<left>,<right>\n")
 *   - Unchanged: PID, battery safety, ramping, kinematics
 * 
 * SERIAL PROTOCOL:
 *   Input commands (from ROS2 cmd_vel_to_arduino):
 *     "M<V>:<W>\n"    — Set linear velocity V (m/s) and angular velocity W (rad/s)
 *     "S<V>\n"         — Set linear velocity only (W=0)
 *     "T<W>\n"         — Set angular velocity only (V=0)
 *   
 *   Output (to ROS2):
 *     "E:<left_cum_ticks>,<right_cum_ticks>\n"  — Cumulative encoder ticks (every 2 cycles)
 *     "RPM:...,Speed:...,Steering:...,Volt:...,Bat%:...\n" — Debug info (every 10 cycles)
 * 
 * =============================================================================
 */

#include <SimpleTimer.h>
SimpleTimer timer;

// --- MOTEURS ---
const int M1_DIR = 7; const int M1_PWM = 6;
const int M2_DIR = 4; const int M2_PWM = 5;

// --- ENCODEURS ---
#define ENC1_A 2 
#define ENC1_B 3
#define ENC2_A 18
#define ENC2_B 19

// --- BATTERIE GRAPHENE 3S (CONFIGURÉE) ---
const int PIN_BATTERIE = A15;
const float VOLT_MAX = 12.6;        // 100% (4.2V/cell)
const float VOLT_MIN = 10.2;        // 0% (3.5V/cell)
const float VOLT_STOP = 10;         // Seuil de coupure d'urgence
const float RATIO_CAPTEUR = 4.8;    // Ratio calibré
bool batterie_critique = false;      // Flag de sécurité

// --- GEOMETRIE ROBOT ---
const double RAYON_ROUE = 0.10; 
const double ENTRAXE = 0.488;    

// --- PARAMETRES PHYSIQUES ---
const int rapport_reducteur = 131;
const int resolution = 64;
const int fe = 128; 
const double dt = 1.0 / fe; 

// --- PID + FEEDFORWARD ---
double Kp = 6.0; double Ki = 35.0; double Kd = 0.05; double Kf = 2.5;  

// --- CIBLES CINEMATIQUES ---
double target_V = 0; double target_W = 0;
double current_V = 0; double current_W = 0;

double accel_V_step = (0.2 / fe); 
double decel_V_step = (0.6 / fe); 
double accel_W_step = (1.5 / fe); 

// Filtre
const double alpha = 0.5; 
double rpm1_filtered = 0; double rpm2_filtered = 0;

// Variables PID
double errorSum1 = 0, lastError1 = 0;
double errorSum2 = 0, lastError2 = 0;
volatile long tick1 = 0; volatile long tick2 = 0;
int plotCounter = 0;

// =============================================================================
// AJOUT POUR NAV2: Compteurs d'encodeur CUMULATIFS (jamais remis à zéro)
// Ces valeurs sont envoyées à ROS2 pour le calcul d'odométrie
// =============================================================================
volatile long cum_tick1 = 0;   // Ticks cumulatifs roue gauche
volatile long cum_tick2 = 0;   // Ticks cumulatifs roue droite
int encoderPubCounter = 0;     // Compteur pour fréquence de publication

// --- INTERRUPTIONS (modifiées pour accumuler les ticks cumulatifs) ---
void ISR_Enc1A() {
  if (digitalRead(ENC1_A) == digitalRead(ENC1_B)) { tick1--; cum_tick1--; }
  else { tick1++; cum_tick1++; }
}
void ISR_Enc1B() {
  if (digitalRead(ENC1_A) == digitalRead(ENC1_B)) { tick1++; cum_tick1++; }
  else { tick1--; cum_tick1--; }
}
void ISR_Enc2A() {
  if (digitalRead(ENC2_A) == digitalRead(ENC2_B)) { tick2--; cum_tick2--; }
  else { tick2++; cum_tick2++; }
}
void ISR_Enc2B() {
  if (digitalRead(ENC2_A) == digitalRead(ENC2_B)) { tick2++; cum_tick2++; }
  else { tick2--; cum_tick2--; }
}

void setMotorPWM(int pwm, int pinPWM, int pinDIR) {
  pwm = constrain(pwm, -255, 255);
  if (abs(pwm) < 5) pwm = 0; 
  if (pwm >= 0) { digitalWrite(pinDIR, LOW); analogWrite(pinPWM, abs(pwm)); }
  else          { digitalWrite(pinDIR, HIGH); analogWrite(pinPWM, abs(pwm)); }
}

// --- FONCTIONS BATTERIE ---
float lireVoltage() {
  int raw = analogRead(PIN_BATTERIE);
  float v = (raw * 5.0 / 1024.0) * RATIO_CAPTEUR;
  
  if (v < VOLT_STOP && v > 5.0) { 
    batterie_critique = true;
  } else if (v > VOLT_MIN + 0.5) { 
    batterie_critique = false;
  }
  return v;
}

int calculerPourcentage(float v) {
  float p = (v - VOLT_MIN) / (VOLT_MAX - VOLT_MIN) * 100.0;
  return constrain((int)p, 0, 100);
}

void boucle_controle() {
  // 1. Lecture Tension et Sécurité
  float vBatCurrent = lireVoltage();
  if (batterie_critique) {
    target_V = 0; target_W = 0;
    current_V = 0; current_W = 0;
    setMotorPWM(0, M1_PWM, M1_DIR);
    setMotorPWM(0, M2_PWM, M2_DIR);
    if (plotCounter % 10 == 0) Serial.println("!!! ALERTE BATTERIE FAIBLE : ARRET !!!");
    return; 
  }

  long t1, t2;
  noInterrupts(); t1 = tick1; tick1 = 0; t2 = tick2; tick2 = 0; interrupts();

  // 2. Calcul RPM
  double rpm1_raw = ((double)t1 / dt) * 60.0 / ((double)resolution * (double)rapport_reducteur);
  double rpm2_raw = ((double)t2 / dt) * 60.0 / ((double)resolution * (double)rapport_reducteur);
  rpm1_filtered = (alpha * rpm1_raw) + ((1.0 - alpha) * rpm1_filtered);
  rpm2_filtered = (alpha * rpm2_raw) + ((1.0 - alpha) * rpm2_filtered);

  // 3. RAMPES
  if (abs(target_V) > abs(current_V)) {
      if (current_V < target_V) current_V += accel_V_step;
      else current_V -= accel_V_step;
  } else {
      if (current_V < target_V) current_V += decel_V_step;
      else current_V -= decel_V_step;
  }
  if (abs(current_V - target_V) < decel_V_step) current_V = target_V;

  if (current_W < target_W) current_W += accel_W_step;
  else if (current_W > target_W) current_W -= accel_W_step;
  if (abs(current_W - target_W) < accel_W_step) current_W = target_W;

  // 4. CINEMATIQUE INVERSE
  double v_roue_gauche = current_V - (current_W * ENTRAXE / 2.0);
  double v_roue_droite = current_V + (current_W * ENTRAXE / 2.0);
  double setpointRPM1 = (v_roue_gauche * 60.0) / (2.0 * PI * RAYON_ROUE);
  double setpointRPM2 = (v_roue_droite * 60.0) / (2.0 * PI * RAYON_ROUE);

  // 5. PID MOTEURS
  double output1 = 0;
  if (target_V == 0 && target_W == 0 && abs(rpm1_filtered) < 1.0) {
     errorSum1 = 0; output1 = 0;
  } else {
     double error1 = setpointRPM1 - rpm1_filtered;
     errorSum1 += error1 * dt;
     errorSum1 = constrain(errorSum1, -4.0, 4.0); 
     output1 = (Kp * error1) + (Ki * errorSum1) + (Kd * (error1 - lastError1)/dt) + (Kf * setpointRPM1);
     lastError1 = error1;
  }

  double output2 = 0;
  if (target_V == 0 && target_W == 0 && abs(rpm2_filtered) < 1.0) {
     errorSum2 = 0; output2 = 0;
  } else {
     double error2 = setpointRPM2 - rpm2_filtered;
     errorSum2 += error2 * dt;
     errorSum2 = constrain(errorSum2, -4.0, 4.0); 
     output2 = (Kp * error2) + (Ki * errorSum2) + (Kd * (error2 - lastError2)/dt) + (Kf * setpointRPM2);
     lastError2 = error2;
  }

  setMotorPWM((int)output1, M1_PWM, M1_DIR);
  setMotorPWM((int)output2, M2_PWM, M2_DIR);

  // =========================================================================
  // 6. PUBLICATION ENCODEURS POUR ROS2 (tous les 2 cycles = 64Hz)
  // =========================================================================
  encoderPubCounter++;
  if (encoderPubCounter >= 2) {
    encoderPubCounter = 0;
    long ct1, ct2;
    noInterrupts(); ct1 = cum_tick1; ct2 = cum_tick2; interrupts();
    Serial.print("E:");
    Serial.print(ct1);
    Serial.print(",");
    Serial.println(ct2);
  }

  // 7. AFFICHAGE DEBUG (toutes les 10 cycles — inchangé)
  plotCounter++;
  if (plotCounter >= 10) {
    plotCounter = 0;
    int pBat = calculerPourcentage(vBatCurrent);
    double avgRPM = (rpm1_filtered + rpm2_filtered) / 2.0;
    double measured_V = (avgRPM * 2.0 * PI * RAYON_ROUE) / 60.0;
    double v_g_mesure = (rpm1_filtered * 2.0 * PI * RAYON_ROUE) / 60.0;
    double v_d_mesure = (rpm2_filtered * 2.0 * PI * RAYON_ROUE) / 60.0;
    double steering_rad_s = (v_d_mesure - v_g_mesure) / ENTRAXE;

    Serial.print("RPM:"); Serial.print(avgRPM); 
    Serial.print(",Speed:"); Serial.print(measured_V, 4); 
    Serial.print(",Steering:"); Serial.print(steering_rad_s, 4);
    Serial.print(",Volt:"); Serial.print(vBatCurrent, 2);
    Serial.print(",Bat%:"); Serial.println(pBat);
  }
}

void parseCommand(String line) {
  line.toUpperCase();
  if (line.startsWith("S")) { target_V = line.substring(1).toFloat(); target_W = 0; }
  if (line.startsWith("T")) { target_W = line.substring(1).toFloat(); target_V = 0; }
  if (line.startsWith("M")) {
      int separator = line.indexOf(':');
      if (separator != -1) {
        target_V = line.substring(1, separator).toFloat();
        target_W = line.substring(separator+1).toFloat();
      }
  }
}

void readCommands() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim(); if(line.length()>0) parseCommand(line);
  }
}

void setup() {
  Serial.begin(57600);
  pinMode(M1_PWM, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_DIR, OUTPUT);
  pinMode(PIN_BATTERIE, INPUT); 
  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), ISR_Enc1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), ISR_Enc1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), ISR_Enc2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), ISR_Enc2B, CHANGE);
  
  timer.setInterval(1000 / fe, boucle_controle);
  Serial.println("ROBOT PRET - NAV2 + SECURITE BATTERIE ACTIVE.");
}

void loop() { timer.run(); readCommands(); }
