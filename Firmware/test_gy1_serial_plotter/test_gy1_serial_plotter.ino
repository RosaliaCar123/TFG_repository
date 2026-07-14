// ============================================================
// TEST — gy1 + accMag1 (astragalo), con y sin cable
//
// Sketch aislado, no toca tobillera_avgs_calibrado.ino.
//
// MODO_LIVE 1: Serial Plotter en vivo. Necesita el cable USB puesto
//              todo el rato (cinta de correr / caminar atado al PC).
//
// MODO_LIVE 0 (por defecto): sin cable. Al encender:
//   1. Espera ESPERA_INICIO_MS (parpadeo RAPIDO del LED) para darte
//      tiempo a ponerte la tobillera y empezar a moverte.
//   2. Graba en RAM durante DURACION_GRABACION_MS (LED FIJO).
//   3. Termina la grabacion (LED APAGADO) y se queda esperando.
//   4. Cuando vuelvas y enchufes el cable, abre el Monitor Serie
//      (115200 baudios) y manda 'd' para volcar el CSV
//      (t_ms,gy1,accMag1). Copialo a Excel/Python para graficarlo.
//
// Reconectar el cable NO resetea la placa ni borra el buffer,
// mientras la tobillera siga alimentada (bateria) todo el tiempo.
// ============================================================

#include "LSM6DS3.h"
#include "Wire.h"

#define FREQ_HZ      100
#define INTERVALO_US 10000

// Cambia a 1 para el modo en vivo (con cable puesto todo el rato)
#define MODO_LIVE 0

#define ESPERA_INICIO_MS      5000   // margen para ponerte la tobillera y arrancar
#define DURACION_GRABACION_MS 10000  // ventana de grabacion tras la espera
#define MAX_MUESTRAS (DURACION_GRABACION_MS / (INTERVALO_US / 1000) + 100)

LSM6DS3 imuAstragalo(I2C_MODE, 0x6A);

#if !MODO_LIVE
struct MuestraDebug { uint16_t t_ms; float gy1; float accMag1; };
MuestraDebug buffer[MAX_MUESTRAS];
uint16_t numMuestras = 0;

enum EstadoTest { ESPERANDO, GRABANDO, LISTO };
EstadoTest estadoTest = ESPERANDO;
uint32_t inicioMs = 0;
uint32_t inicioGrabacionMs = 0;

void volcarBuffer() {
    Serial.println("t_ms,gy1,accMag1");
    for (uint16_t i = 0; i < numMuestras; i++) {
        Serial.print(buffer[i].t_ms);   Serial.print(",");
        Serial.print(buffer[i].gy1, 3); Serial.print(",");
        Serial.println(buffer[i].accMag1, 3);
    }
    Serial.print("FIN ("); Serial.print(numMuestras); Serial.println(" muestras)");
}
#endif

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);

#if MODO_LIVE
    while (!Serial) {} // en modo en vivo si tiene sentido esperar al cable
#endif

    if (imuAstragalo.begin() != 0) {
        // sin cable no se ve el Serial: el LED parpadeando avisa del fallo
        while (1) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(200);
        }
    }

#if MODO_LIVE
    Serial.println("t_ms,gy1,accMag1");
#else
    inicioMs = millis();
    Serial.println("Grabacion sin cable armada. Parpadeo rapido = esperando; fijo = grabando.");
#endif
}

void loop() {
    static unsigned long ultimaMuestra = 0;
    unsigned long ahora = micros();
    if (ahora - ultimaMuestra < INTERVALO_US) {
#if !MODO_LIVE
        if (estadoTest == LISTO && Serial.available() && Serial.read() == 'd') {
            volcarBuffer();
        }
#endif
        return;
    }
    ultimaMuestra = ahora;

    float ax1 = imuAstragalo.readFloatAccelX();
    float ay1 = imuAstragalo.readFloatAccelY();
    float az1 = imuAstragalo.readFloatAccelZ();
    float gy1 = imuAstragalo.readFloatGyroY();
    float accMag1 = sqrt(ax1*ax1 + ay1*ay1 + az1*az1);

#if MODO_LIVE
    Serial.print("gy1:");      Serial.print(gy1);
    Serial.print(",accMag1:"); Serial.println(accMag1);
#else
    switch (estadoTest) {
        case ESPERANDO:
            digitalWrite(LED_BUILTIN, (millis() / 150) % 2); // parpadeo rapido
            if (millis() - inicioMs > ESPERA_INICIO_MS) {
                estadoTest = GRABANDO;
                inicioGrabacionMs = millis();
                numMuestras = 0;
            }
            break;

        case GRABANDO:
            digitalWrite(LED_BUILTIN, HIGH);
            if (numMuestras < MAX_MUESTRAS) {
                buffer[numMuestras].t_ms     = (uint16_t)(millis() - inicioGrabacionMs);
                buffer[numMuestras].gy1      = gy1;
                buffer[numMuestras].accMag1  = accMag1;
                numMuestras++;
            }
            if (millis() - inicioGrabacionMs > DURACION_GRABACION_MS
                || numMuestras >= MAX_MUESTRAS) {
                estadoTest = LISTO;
                digitalWrite(LED_BUILTIN, LOW);
            }
            break;

        case LISTO:
            if (Serial.available() && Serial.read() == 'd') {
                volcarBuffer();
            }
            break;
    }
#endif
}
