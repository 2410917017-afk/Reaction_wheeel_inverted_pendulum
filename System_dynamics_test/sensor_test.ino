// ================================================================
// Reaction Wheel Pendulum — Lectura de sensores
// ESP32 + AS5600 (velocidad rueda) + Encoder 3806 (ángulo péndulo)
// ================================================================

#include <Wire.h>

// ── Pines ────────────────────────────────────────────────────────
#define ENC_A       18      // Canal A encoder 3806
#define ENC_B       19      // Canal B encoder 3806
#define I2C_SDA     21
#define I2C_SCL     22

// ── Parámetros del encoder 3806 ──────────────────────────────────
#define PPR         600     // Pulsos por revolución
#define CPR         2400    // Cuentas por revolución (×4 cuadratura)

// ── Parámetros del AS5600 ─────────────────────────────────────────
#define AS5600_ADDR     0x36
#define AS5600_RAW_ANG  0x0C  // Registro ángulo crudo (2 bytes)
#define AS5600_STATUS   0x0B  // Registro de estado
#define AS5600_AGC      0x1A  // Control de ganancia automático

// ── Variables del encoder 3806 (volatile por ser usadas en ISR) ──
volatile long  enc_count    = 0;
volatile bool  enc_A_last   = false;

// ── Variables del AS5600 ──────────────────────────────────────────
int16_t  as_angle_raw_prev  = 0;
float    as_angle_deg       = 0.0;
float    wheel_velocity     = 0.0;   // [°/s]

// ── Tiempo ────────────────────────────────────────────────────────
unsigned long t_last_sensor = 0;
unsigned long t_last_print  = 0;
const uint16_t DT_SENSOR_US = 2000;   // 500 Hz loop de control
const uint16_t DT_PRINT_MS  = 100;    // 10 Hz impresión serial


// ================================================================
//  ISR — Interrupción del encoder 3806
//  Se ejecuta en cada flanco del canal A
// ================================================================
void IRAM_ATTR ISR_encoder() {
    bool A = digitalRead(ENC_A);
    bool B = digitalRead(ENC_B);
    // Cuadratura: si A y B iguales → sentido horario
    if (A == B) enc_count++;
    else        enc_count--;
}


// ================================================================
//  AS5600 — Funciones de lectura
// ================================================================

// Lee el ángulo crudo de 12 bits (0–4095)
int16_t as5600_read_raw() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_RAW_ANG);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 2);

    if (Wire.available() < 2) return -1;

    uint8_t high = Wire.read();
    uint8_t low  = Wire.read();
    return ((high & 0x0F) << 8) | low;  // 12 bits válidos
}

// Verifica que el imán está correctamente posicionado
bool as5600_check_magnet() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_STATUS);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDR, 1);

    if (!Wire.available()) return false;
    uint8_t status = Wire.read();

    // Bit 5: MD (Magnet Detected)
    // Bit 4: ML (Too weak)  Bit 3: MH (Too strong)
    bool detected = (status >> 5) & 0x01;
    bool too_weak = (status >> 4) & 0x01;
    bool too_strong = (status >> 3) & 0x01;

    if (too_weak)   Serial.println("⚠ AS5600: imán muy débil o lejos");
    if (too_strong) Serial.println("⚠ AS5600: imán muy fuerte o cerca");

    return detected && !too_weak && !too_strong;
}

// Calcula velocidad angular con manejo de desbordamiento (wrap-around)
float as5600_get_velocity(int16_t raw_now, int16_t raw_prev, float dt_s) {
    int16_t delta = raw_now - raw_prev;

    // Manejar cruce de 0/4095 (vuelta completa)
    if (delta >  2048) delta -= 4096;
    if (delta < -2048) delta += 4096;

    // Convertir a °/s: 4096 cuentas = 360°
    return (delta * 360.0f / 4096.0f) / dt_s;
}


// ================================================================
//  Funciones de lectura del péndulo (encoder 3806)
// ================================================================

// Ángulo del péndulo en grados
float get_pendulum_angle() {
    long count = enc_count;  // lectura atómica suficiente para float
    return (count * 360.0f) / CPR;
}

// Velocidad angular del péndulo en °/s (derivada numérica)
float get_pendulum_velocity(float angle_now, float angle_prev, float dt_s) {
    return (angle_now - angle_prev) / dt_s;
}

// Reset del encoder al inicio (posición vertical = 0)
void encoder_reset() {
    enc_count = 0;
}


// ================================================================
//  INICIALIZACIÓN
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n========================================");
    Serial.println("  Reaction Wheel Pendulum — Inicio");
    Serial.println("========================================");

    // ── 1. I2C ───────────────────────────────────────────────────
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);  // 400 kHz fast mode
    Serial.println("[1/4] I2C iniciado a 400kHz");

    // ── 2. AS5600 ────────────────────────────────────────────────
    Serial.print("[2/4] Verificando AS5600... ");
    delay(100);  // tiempo de arranque del sensor

    bool magnet_ok = as5600_check_magnet();
    if (magnet_ok) {
        Serial.println("✓ Imán detectado correctamente");
    } else {
        Serial.println("✗ ERROR — verifica posición del imán");
        Serial.println("    El imán debe estar a 0.5–3mm del sensor");
        Serial.println("    Centrado sobre la cara del AS5600");
        // No bloquea — continúa para depuración
    }

    // Leer valor inicial
    as_angle_raw_prev = as5600_read_raw();

    // ── 3. Encoder 3806 ──────────────────────────────────────────
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A),
                    ISR_encoder, CHANGE);
    Serial.println("[3/4] Encoder 3806 iniciado");

    // ── 4. Posición inicial ──────────────────────────────────────
    Serial.println("[4/4] Lleva el péndulo a posición vertical...");
    Serial.println("      Presiona ENTER cuando esté en posición");

    // Esperar comando serial para hacer cero
    while (!Serial.available()) delay(10);
    Serial.read();
    encoder_reset();
    Serial.println("      ✓ Posición cero establecida");

    t_last_sensor = micros();
    t_last_print  = millis();

    Serial.println("\n--- Iniciando lectura ---");
    Serial.println("theta[°]\ttheta_dot[°/s]\tphi_dot[°/s]");
}


// ================================================================
//  LOOP PRINCIPAL
// ================================================================
void loop() {

    unsigned long t_now_us = micros();

    // ── Loop de sensores a 500 Hz ─────────────────────────────────
    if (t_now_us - t_last_sensor >= DT_SENSOR_US) {

        float dt = (t_now_us - t_last_sensor) * 1e-6f;  // en segundos
        t_last_sensor = t_now_us;

        // Leer ángulo péndulo (encoder 3806)
        static float theta_prev = 0;
        float theta = get_pendulum_angle();
        float theta_dot = get_pendulum_velocity(theta, theta_prev, dt);
        theta_prev = theta;

        // Leer velocidad rueda (AS5600)
        int16_t raw_now = as5600_read_raw();
        if (raw_now >= 0) {
            wheel_velocity = as5600_get_velocity(raw_now,
                                                  as_angle_raw_prev,
                                                  dt);
            as_angle_raw_prev = raw_now;
        }


        // ── Imprimir a 10 Hz ─────────────────────────────────────
        if (millis() - t_last_print >= DT_PRINT_MS) {
            t_last_print = millis();

            Serial.print(theta, 3);         Serial.print("\t");
            Serial.print(theta_dot, 2);     Serial.print("\t");
            Serial.println(wheel_velocity, 2);
        }
    }
}
