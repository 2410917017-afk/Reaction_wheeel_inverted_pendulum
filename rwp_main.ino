// ================================================================
// Reaction Wheel Pendulum — Controlador LQR Completo
// ESP32 + AS5600 + Encoder 3806 + Driver L298N/TB6612
// ================================================================

#include <Wire.h>

// ================================================================
//  CONFIGURACIÓN 
// ================================================================

// ── Ganancias LQR  ──────
// K = [K_theta, K_thetadot, K_phidot]
const float K_theta    = 15.0;    // ganancia en ángulo
const float K_thetadot = 8.0;     // ganancia en velocidad péndulo
const float K_phidot   = 2.0;     // ganancia en velocidad rueda

// ── Límites del sistema ──────────────────────────────────────────
const float THETA_MAX_DEG   = 25.0;  // ángulo máximo antes de apagar
const float VMAX            = 18.0;  // voltaje máximo del motor [V]
const float PWM_DEADZONE    = 18;    // PWM mínimo donde motor arranca
                                     // (identificar experimentalmente)
const float CONTROL_FREQ_HZ = 500;   // frecuencia del loop [Hz]

// ── Filtro pasa bajas para velocidades ───────────────────────────
const float ALPHA_THETA = 0.20;    // suavizado de θ̇  (0=más suave)
const float ALPHA_PHI   = 0.15;    // suavizado de φ̇  (0=más suave)

// ── Pines ────────────────────────────────────────────────────────
#define ENC_A     18
#define ENC_B     19
#define I2C_SDA   21
#define I2C_SCL   22

// Driver L298N
#define MOTOR_IN1   25    // dirección A
#define MOTOR_IN2   26    // dirección B
#define MOTOR_ENA   27    // PWM enable  (canal LEDC 0)

// O si usas TB6612:
// #define MOTOR_AIN1  25
// #define MOTOR_AIN2  26
// #define MOTOR_PWMA  27
// #define MOTOR_STBY  14   // pull HIGH para habilitar

// ── AS5600 ───────────────────────────────────────────────────────
#define AS5600_ADDR    0x36
#define AS5600_RAW_ANG 0x0C
#define AS5600_STATUS  0x0B

// ── Encoder 3806 ─────────────────────────────────────────────────
#define CPR  2400    // cuentas por revolución (600 PPR × 4)

// ── PWM ESP32 (LEDC) ─────────────────────────────────────────────
#define PWM_CHANNEL   0
#define PWM_FREQ_HZ   20000   // 20kHz — inaudible
#define PWM_BITS      8       // resolución 0-255


// ================================================================
//  VARIABLES GLOBALES
// ================================================================

// Encoder 3806
volatile long enc_count = 0;

// Estados del sistema
float theta        = 0;     // ángulo péndulo [°]
float theta_dot    = 0;     // velocidad péndulo [°/s]
float phi_dot      = 0;     // velocidad rueda [°/s]

// Filtros
float theta_dot_f  = 0;     // θ̇ filtrada
float phi_dot_f    = 0;     // φ̇ filtrada

// Control
float u_control    = 0;     // señal de control [V]
bool  control_activo = false;
bool  sistema_ok     = true;

// Tiempo
unsigned long t_last_us   = 0;
unsigned long t_last_print = 0;
const float   DT_S = 1.0f / CONTROL_FREQ_HZ;
const uint32_t DT_US = (uint32_t)(DT_S * 1e6);

// AS5600
int16_t as_raw_prev = 0;


// ================================================================
//  ISR — Encoder 3806
// ================================================================
void IRAM_ATTR ISR_encoder() {
    bool A = digitalRead(ENC_A);
    bool B = digitalRead(ENC_B);
    if (A == B) enc_count++;
    else        enc_count--;
}


// ================================================================
//  MOTOR
// ================================================================

void motor_init() {
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(MOTOR_ENA, PWM_CHANNEL);
    motor_stop();
}

void motor_stop() {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);
}

// Aplica voltaje al motor con compensación de zona muerta
// u en Voltios, rango [-VMAX, +VMAX]
void motor_set_voltage(float u) {

    if (!sistema_ok) {
        motor_stop();
        return;
    }

    // Saturación
    u = constrain(u, -VMAX, VMAX);

    // Zona muerta de control — debajo de umbral el péndulo
    // está tan cerca de cero que no vale la pena actuar
    if (abs(u) < 0.3) {
        motor_stop();
        return;
    }

    // Convertir voltaje a PWM con compensación de zona muerta
    // PWM = zona_muerta + (|u|/Vmax) × (255 - zona_muerta)
    int pwm = (int)(PWM_DEADZONE +
              (abs(u) / VMAX) * (255 - PWM_DEADZONE));
    pwm = constrain(pwm, 0, 255);

    if (u > 0) {
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
    } else {
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, HIGH);
    }

    ledcWrite(PWM_CHANNEL, pwm);
}


// ================================================================
//  AS5600
// ================================================================

int16_t as5600_read_raw() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_RAW_ANG);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() < 2) return -1;
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    return ((hi & 0x0F) << 8) | lo;
}

bool as5600_magnet_ok() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_STATUS);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 1);
    if (!Wire.available()) return false;
    uint8_t s = Wire.read();
    return ((s >> 5) & 1) && !((s >> 4) & 1) && !((s >> 3) & 1);
}

// Velocidad con manejo de wrap-around 0/4095
float as5600_velocity(int16_t raw_now, int16_t raw_prev, float dt) {
    int16_t delta = raw_now - raw_prev;
    if (delta >  2048) delta -= 4096;
    if (delta < -2048) delta += 4096;
    return (delta * 360.0f / 4096.0f) / dt;
}


// ================================================================
//  LECTURA DE ESTADOS
// ================================================================

float get_theta() {
    return ((float)enc_count * 360.0f) / CPR;
}

// ── Seguridad — apaga si el péndulo cae demasiado ────────────────
void check_safety(float angle) {
    if (abs(angle) > THETA_MAX_DEG) {
        sistema_ok = false;
        motor_stop();
        control_activo = false;
    }
}


// ================================================================
//  INICIALIZACIÓN
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║   Reaction Wheel Pendulum — v1.0     ║");
    Serial.println("╚══════════════════════════════════════╝\n");

    // ── 1. Motor (apagado primero por seguridad) ──────────────────
    motor_init();
    Serial.println("[1/5] ✓ Motor inicializado y apagado");

    // ── 2. I2C ────────────────────────────────────────────────────
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    Serial.println("[2/5] ✓ I2C a 400kHz");

    // ── 3. AS5600 ─────────────────────────────────────────────────
    Serial.print("[3/5] AS5600... ");
    delay(100);
    if (as5600_magnet_ok()) {
        as_raw_prev = as5600_read_raw();
        Serial.println("✓ Imán OK");
    } else {
        Serial.println("⚠ Imán no detectado — verifica distancia");
    }

    // ── 4. Encoder 3806 ───────────────────────────────────────────
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), ISR_encoder, CHANGE);
    Serial.println("[4/5] ✓ Encoder 3806 con interrupciones");

    // ── 5. Cero del péndulo ───────────────────────────────────────
    Serial.println("[5/5] Posiciona el péndulo en VERTICAL");
    Serial.println("      y envía cualquier carácter por Serial");
    Serial.println("      para establecer el cero...\n");

    while (!Serial.available()) delay(10);
    while (Serial.available()) Serial.read();  // limpiar buffer

    enc_count = 0;
    as_raw_prev = as5600_read_raw();
    theta_dot_f = 0;
    phi_dot_f   = 0;

    Serial.println("✓ Cero establecido\n");

    // Mostrar ganancias activas
    Serial.println("── Ganancias LQR ──────────────────────");
    Serial.print("K_theta    = "); Serial.println(K_theta, 4);
    Serial.print("K_thetadot = "); Serial.println(K_thetadot, 4);
    Serial.print("K_phidot   = "); Serial.println(K_phidot, 4);
    Serial.println("───────────────────────────────────────");
    Serial.println("\nEnvía 'S' para INICIAR control");
    Serial.println("Envía 'P' para PARAR");
    Serial.println("Envía 'R' para RESETEAR cero\n");

    Serial.println("theta\ttheta_d\tphi_d\tu[V]\tOK");

    t_last_us    = micros();
    t_last_print = millis();
}


// ================================================================
//  LOOP PRINCIPAL
// ================================================================
void loop() {

    // ── Comandos seriales ─────────────────────────────────────────
    if (Serial.available()) {
        char cmd = toupper(Serial.read());

        if (cmd == 'S') {
            sistema_ok     = true;
            control_activo = true;
            enc_count      = 0;
            phi_dot_f      = 0;
            theta_dot_f    = 0;
            Serial.println("▶ Control ACTIVO");

        } else if (cmd == 'P') {
            control_activo = false;
            motor_stop();
            Serial.println("■ Control PAUSADO");

        } else if (cmd == 'R') {
            enc_count   = 0;
            as_raw_prev = as5600_read_raw();
            sistema_ok  = true;
            Serial.println("↺ Cero restablecido");
        }
    }

    // ── Loop de control ───────────────────────────────────────────
    unsigned long t_now = micros();
    if (t_now - t_last_us < DT_US) return;  // esperar el período

    float dt = (t_now - t_last_us) * 1e-6f;
    t_last_us = t_now;

    // Limitar dt por si hubo retraso (protección)
    dt = constrain(dt, 0.0005f, 0.005f);

    // ── Leer estados ──────────────────────────────────────────────
    static float theta_prev = 0;

    theta = get_theta();

    // Velocidad del péndulo — derivada + filtro
    float theta_dot_raw = (theta - theta_prev) / dt;
    theta_dot_f = ALPHA_THETA * theta_dot_raw +
                  (1 - ALPHA_THETA) * theta_dot_f;
    theta_prev = theta;

    // Velocidad de la rueda — AS5600 + filtro
    int16_t raw_now = as5600_read_raw();
    if (raw_now >= 0) {
        float phi_dot_raw = as5600_velocity(raw_now, as_raw_prev, dt);
        phi_dot_f   = ALPHA_PHI * phi_dot_raw +
                      (1 - ALPHA_PHI) * phi_dot_f;
        as_raw_prev = raw_now;
    }

    // ── Verificar seguridad ───────────────────────────────────────
    check_safety(theta);

    if (!sistema_ok) {
        motor_stop();
        if (millis() - t_last_print > 500) {
            t_last_print = millis();
            Serial.println("⛔ SISTEMA CAÍDO — envía 'R' para resetear");
        }
        return;
    }

    // ── Controlador LQR ──────────────────────────────────────────
    if (control_activo) {

        // Convertir a radianes para que coincida con las
        // ganancias calculadas en MATLAB (que usa radianes)
        float theta_rad    = theta     * DEG_TO_RAD;
        float thetadot_rad = theta_dot_f * DEG_TO_RAD;
        float phidot_rad   = phi_dot_f   * DEG_TO_RAD;

        // u = -K·x
        u_control = -(K_theta    * theta_rad    +
                      K_thetadot * thetadot_rad +
                      K_phidot   * phidot_rad);

        motor_set_voltage(u_control);

    } else {
        motor_stop();
        u_control = 0;
    }

    // ── Telemetría serial a 10 Hz ─────────────────────────────────
    if (millis() - t_last_print >= 100) {
        t_last_print = millis();

        Serial.print(theta, 3);       Serial.print("\t");
        Serial.print(theta_dot_f, 2); Serial.print("\t");
        Serial.print(phi_dot_f, 2);   Serial.print("\t");
        Serial.print(u_control, 3);   Serial.print("\t");
        Serial.println(control_activo ? "ON" : "OFF");
    }
}
