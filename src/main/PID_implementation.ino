// Implementación de sistema de control por PID anidado en la rueda de reacción con ESP32, Motor JGA-25, driver L293D
#include <Arduino.h>
#include <PID_v1.h>
#include <ESP32Encoder.h>

// ── Pines ─────────────────────────────────────────────────────────
const int Encoder1A = 18;
const int Encoder1B = 19;
const int Encoder2A = 22;
const int Encoder2B = 23;
const int dirA      = 15;
const int dirB      = 4;
const int enable    = 5;

volatile long encoderCount = 0; // Contador de pulsos
unsigned long lastMicros = 0;   // Para debounce
const unsigned long debounceTime = 500; 

// ── Constantes ────────────────────────────────────────────────────
const float CPR1     = 1200.0f;   // 600 PPR para HalfQuad
const float CPR2     = 980.0f;    // 490 PPR 
const int   DT_MS    = 10;        // período del loop 
const float DT_S     = DT_MS / 1000.0f;
const float THETA_MAX = 100.0f;
const int   PWM_DEAD  = 20;       // zona muerta 

// ── Variables de estado ───────────────────────────────────────────
float theta      = 0, theta_prev = 0, 
float phi        = 0, phi_prev   = 0, phi_dot   = 0;

// ── PID 
double Setpoint_ext = 0,  Input_ext = 0,  Output_ext = 0;
double Setpoint_int = 0,  Input_int = 0,  Output_int = 0;

// Ganancias — empezar en 0 y subir durante sintonización
double Kp_ext = 160.0, Ki_ext = 0, Kd_ext = 2.0;
double Kp_int = 1.0, Ki_int = 0.4, Kd_int = 0.1;

PID PID_ext(&Input_ext, &Output_ext, &Setpoint_ext,
            Kp_ext, Ki_ext, Kd_ext, DIRECT);

PID PID_int(&Input_int, &Output_int, &Setpoint_int,
            Kp_int, Ki_int, Kd_int, DIRECT);

ESP32Encoder encoderA;
ESP32Encoder encoderB;

// ── Timing ────────────────────────────────────────────────────────
unsigned long t_last_control = 0;
unsigned long t_last_serial  = 0;


// ── Motor ─────────────────────────────────────────────────────────
void motor_apply(float u) {
  //int pwm = (int)constrain(fabsf(u), 0, 255);  // ← valor absoluto
  int pwm = (int)u;
 
  if (pwm > 35) {
    digitalWrite(dirA, HIGH);
    digitalWrite(dirB, LOW);
    analogWrite(enable, pwm);
  } else if (pwm < -35) {
    digitalWrite(dirA, LOW);
    digitalWrite(dirB, HIGH);
    analogWrite(enable, -pwm);
  } else {
    analogWrite(enable, 0);
  }
    
}



//  SETUP
void setup() {
  Serial.begin(115200);
  delay(500);

  // Motor apagado primero
  pinMode(dirA,   OUTPUT);
  pinMode(dirB,   OUTPUT);
  pinMode(enable, OUTPUT);
  digitalWrite(dirA, LOW);
  digitalWrite(dirB, LOW);
  analogWrite(enable, 0);

  // Encoders

  ESP32Encoder :: useInternalWeakPullResistors = puType :: up;
  encoderA.attachHalfQuad(18, 19);
  encoderB.attachHalfQuad(22, 23);

  encoderA.clearCount();
  encoderB.clearCount();
  
  // PID — período de cómputo igual al loop
  PID_ext.SetMode(AUTOMATIC);
  PID_int.SetMode(AUTOMATIC);
  PID_ext.SetSampleTime(DT_MS);    // sincronizar con tu loop
  PID_int.SetSampleTime(DT_MS);

  // Límites de salida
  PID_ext.SetOutputLimits(-800, 800);  // referencia de vel. rueda [°/s]
  PID_int.SetOutputLimits(-255, 255);  // PWM al motor

  // Setpoint externo siempre en 0 (queremos θ=0)
  Setpoint_ext = 0;
  Setpoint_int = 0;

  Serial.println("Posiciona péndulo en vertical y envía cualquier tecla");
  while (!Serial.available()) delay(10);
  Serial.read();
  Serial.println("✓ Cero establecido. Esperando...");
  Serial.println("Comandos: S=iniciar  P=pausar  R=reset");

  t_last_control = millis();
  t_last_serial  = millis();
  
}


//  LOOP
bool activo = false;

void loop() {

  // ── Comandos seriales ─────────────────────────────────────
  if (Serial.available()) {
    String lectura = Serial.readStringUntil('\n');
    lectura.trim();

    int index = lectura.indexOf(',');
    String cmd = lectura.substring(0, index);
    double valor = lectura.substring(index+1).toDouble();
    
    if (cmd == "S") {
      activo = false; // Pausar un momento
      
      // 1. Cero en encoders
      encoderA.setCount(0);
      encoderB.setCount(0);

      theta = 0; phi = 0;
      theta_prev = 0; phi_prev = 0;
      theta_dot = 0; phi_dot = 0;
      Input_ext = 0; Input_int = 0;

      // 3. Reiniciar la memoria integral de los PIDs (Bumpless Transfer)
      PID_ext.SetMode(MANUAL); 
      PID_int.SetMode(MANUAL);
      Output_ext = 0; 
      Output_int = 0;
      PID_ext.SetMode(AUTOMATIC); 
      PID_int.SetMode(AUTOMATIC);

      activo = true;  // Arrancar limpio
      Serial.println("▶ Activo (Limpio)");
    } else if (cmd == "P"){
      activo = false; analogWrite(enable, 0); Serial.println("■ Pausado");
    } else if (cmd == "R"){
      // 1. Cero en encoders
      encoderA.setCount(0);
      encoderB.setCount(0);
      
      theta = 0; phi = 0;
      theta_prev = 0; phi_prev = 0;
      theta_dot = 0; phi_dot = 0;
      Input_ext = 0; Input_int = 0;

      // 3. Reiniciar la memoria integral de los PIDs (Bumpless Transfer)
      PID_ext.SetMode(MANUAL); 
      PID_int.SetMode(MANUAL);
      Output_ext = 0; 
      Output_int = 0;
      PID_ext.SetMode(AUTOMATIC); 
      PID_int.SetMode(AUTOMATIC);
      Serial.println("↺ Reset");
    } else if (cmd == "KP"){
      Kp_ext = valor;
      PID_ext.SetTunings(Kp_ext, Ki_ext, Kd_ext);
    } else if (cmd == "KD"){
      Kd_ext = valor;
      PID_ext.SetTunings(Kp_ext, Ki_ext, Kd_ext);
    } else if (cmd == "KI"){
      Ki_ext = valor;
      PID_ext.SetTunings(Kp_ext, Ki_ext, Kd_ext);
    }
    
  }

  // ── Loop de control a 100 Hz ──────────────────────────────
  if (millis() - t_last_control < DT_MS) return;
  t_last_control = millis();

  // Leer encoders
  long c1 = encoderA.getCount();
  long c2 = encoderB.getCount();

  theta = (c1 * 360.0f) / CPR1;
  phi   = (c2 * 360.0f) / CPR2;

  // Velocidades con filtro pasa bajas
  //theta_dot =  0.2f * ((theta - theta_prev) / DT_S) + 0.8f * theta_dot;
  //theta_dot = (theta - theta_prev) / DT_S;
  phi_dot   =  0.2f * ((phi   - phi_prev)   / DT_S) + 0.8f * phi_dot;
  //phi_dot = (phi   - phi_prev)   / DT_S;
  //theta_prev = theta;
  phi_prev   = phi;

  if (!activo) return;

  if (fabsf(theta) > 25 ){
    activo = false; analogWrite(enable, 0); Serial.println("■ Pausado");
  }

  // Actualizar entradas PID 
  Input_ext = theta;
  Input_int = phi_dot;

  // PID externo: θ → referencia de velocidad de rueda
  PID_ext.Compute();
  Setpoint_int = -Output_ext;   // ← cascada: salida ext es setpoint int

  // PID interno: φ̇ → PWM del motor
  PID_int.Compute();

  // Aplicar al motor
  motor_apply((float)Output_int);

  // ── Serial a 10 Hz ────────────────────────────────────────
  if (millis() - t_last_serial >= 100) {
    t_last_serial = millis();

    Serial.print(Kp_ext); Serial.print("\t"); Serial.print("\t");
    Serial.print(Ki_ext); Serial.print("\t"); Serial.print("\t");
    Serial.print(Kd_ext); Serial.print("\t"); Serial.print("\t");

    Serial.print(theta, 2);         Serial.print("\t"); Serial.print("\t");
    //Serial.print(phi_dot, 2);       Serial.print("\t"); Serial.print("\t");
    //Serial.print(Output_ext, 1);    Serial.print("\t"); Serial.print("\t");
    Serial.println(Output_int, 1);
  }
}
