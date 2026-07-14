// ============================================================
// TOBILLERA TFG — AVGS + Calibracion estatica
//
// Flujo de uso:
//   1. Usuario manda 0x01 desde la app
//   2. Se queda 5 segundos QUIETO (pie plano en el suelo)
//   3. El micro promedia los angulos y guarda offsets
//   4. Empieza a detectar pisadas normalmente
//   5. Los angulos guardados son RELATIVOS a la posicion neutra
//  *** Adicionalmente
//   6. Se para a los 10 segundos para poder hacerlo sola 
//   7. ELimina las dos primeras y las dos segundas 
//
// Deteccion por giroscopio (metodo AVGS)
//  Angular Velocity-based Gait Segmentation
// Se divide en tres fases la pisada: 
// 1. vuelo: pico grande de rotacion: hacia delante
// 2. Contacto: la señal cae bruscamente cuando el pie impacta en el suelo
// 3. Apoyo: señal estable cerca de cero 
// 4. Despegue: el pie vuelve a rotar- la velocidad sube  

// Se calculan los angulos utilizando los algoritmos de madgwick que integra la vel angular(lo que da la imu) para calcular angulos
// Estos filtros s etienen que llmaar constantemente para que no pierdan la referencia, como los gps 

// Referencias:
//   - Ruiz-Ruiz et al. (2024) — algoritmo AVGS
//   - Luo et al. (2024)       — refinamiento zero-crossing
//   - Fadillioglu et al. (2020) — validacion en carrera
// ============================================================

#include "LSM6DS3.h"
#include <Adafruit_LSM6DSOX.h>
#include <MadgwickAHRS.h>
#include <ArduinoBLE.h>
#include "Wire.h"

// ============================================================
// CONFIGURACION
// ============================================================
#define FREQ_HZ          100
#define INTERVALO_US     10000
#define MAX_PISADAS      50
#define MAX_MUESTRAS     30
#define NUM_INSTANTES    9
#define UMBRAL_RUIDO     0.04f

// --- Deteccion por giroscopio (metodo AVGS) ---
#define UMBRAL_MSW_DPS       190.0f // velocidad angular durante vuelo
//#define UMBRAL_APOYO_DPS     100.0f // cae y empieza a poyar 
#define DURACION_MINIMA_MS   150// descarta pisadas inferiores a 150ms: 6.67 Hz de zancada
#define DURACION_MAXIMA_MS   500 
#define BLOQUEO_MS           100 //evita oscilaciones 100ms despues de una pisada (filtra)

// --- Calibracion estatica ---
#define TIEMPO_CALIBRACION_MS   5000    // 5 segundos quieto
#define MUESTRAS_CALIBRACION    500     // 500 muestras a 100Hz = 5s

const float INSTANTES[NUM_INSTANTES] = {0, 10, 20, 27, 35, 67, 92, 98, 100}; //instantes de interes 

// ============================================================
// ESTRUCTURAS
// ============================================================
struct Muestra {
    int16_t velX1, velY1, velZ1;
    int16_t posX1, posY1, posZ1;
    int16_t velX2, velY2, velZ2;
    int16_t posX2, posY2, posZ2;
    uint16_t t_ms;
};

struct Pisada {
    Muestra muestras[MAX_MUESTRAS]; //a 100hz
    Muestra instantes[NUM_INSTANTES]; //los 9 instantes
    uint8_t  numMuestras;
    uint16_t duracion_ms;
};

// ============================================================
// VARIABLES GLOBALES
// ============================================================
LSM6DS3           imuAstragalo(I2C_MODE, 0x6A);
Adafruit_LSM6DSOX imuMeta;

Madgwick filtroAst;
Madgwick filtroMeta;

Pisada   sesion[MAX_PISADAS];
uint8_t  numPisadas   = 0;
bool     sesionActiva = false;
bool     calibrado    = false;
uint32_t inicioSesionMs = 0;

// --- Offsets de calibracion ---
float rollOffAst  = 0, pitchOffAst  = 0, yawOffAst  = 0;
float rollOffMeta = 0, pitchOffMeta = 0, yawOffMeta = 0;

// --- Maquina de estados AVGS ---
enum EstadoPisada {
    ESPERANDO_MSW,
    ESPERANDO_IC,
    EN_APOYO
};

EstadoPisada estado = ESPERANDO_MSW; // se empieza esperando un cambio en el giroscopio 
uint32_t inicioPisada = 0;
uint32_t finPisadaMs  = 0;


struct EstadoIMU {
    float velX = 0, velY = 0, velZ = 0;
    float roll = 0, pitch = 0, yaw = 0;
    unsigned long tiempoAnterior = 0;
};

EstadoIMU estadoAst;
EstadoIMU estadoMeta;

// BLE
BLEService        datoService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic pisadaChar("19B10001-E8F2-537E-4F6C-D104768A1214",
                              BLERead | BLENotify, 50);
BLECharacteristic cmdChar("19B10002-E8F2-537E-4F6C-D104768A1214",
                           BLEWrite, 1);

// ============================================================
// FUNCIONES
// ============================================================

void resetEstado(EstadoIMU& e) {
    e.velX = 0; e.velY = 0; e.velZ = 0;
    e.roll = 0; e.pitch = 0; e.yaw = 0;
    e.tiempoAnterior = micros();
}

void actualizarEstado(EstadoIMU& e, Madgwick& filtro,
                      float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float rollOff, float pitchOff, float yawOff) {

    unsigned long ahora = micros();
    float dt = (ahora - e.tiempoAnterior) / 1000000.0f;
    e.tiempoAnterior = ahora;
    if (dt <= 0 || dt > 0.1f) return;

    filtro.updateIMU(gx, gy, gz, ax, ay, az);
    // psasr a radianes:
    float roll  = filtro.getRoll()  * PI / 180.0f;
    float pitch = filtro.getPitch() * PI / 180.0f;
    // calcular el componente de la gravedad que afecta a cada eje 
    float gravX =  sin(pitch);
    float gravY = -sin(roll) * cos(pitch);
    float gravZ =  cos(roll) * cos(pitch);
    //calcular la aceleracion en eje quitando la gravedad en m/s2
    float linAx = (ax - gravX) * 9.81f;
    float linAy = (ay - gravY) * 9.81f;
    float linAz = (az - gravZ) * 9.81f;
    //ignorar limites de ruido 
    if (fabsf(linAx) < UMBRAL_RUIDO) linAx = 0;
    if (fabsf(linAy) < UMBRAL_RUIDO) linAy = 0;
    if (fabsf(linAz) < UMBRAL_RUIDO) linAz = 0;
    // intefrar para sacar la velocidad 
    e.velX += linAx * dt;
    e.velY += linAy * dt;
    e.velZ += linAz * dt;

    // Angulos RELATIVOS: restar el offset de calibracion
    e.roll  = filtro.getRoll()  - rollOff;
    e.pitch = filtro.getPitch() - pitchOff;
    e.yaw   = filtro.getYaw()   - yawOff;
}

// aproxiar a dos decimales para reducir a 2bytes: 34767 son limites 
inline int16_t toInt16Vel(float v) { return (int16_t)constrain(v * 100.0f, -32767, 32767); }
inline int16_t toInt16Ang(float a) { return (int16_t)constrain(a * 100.0f, -32767, 32767); }

void seleccionarInstantes(Pisada& p) {
    for (uint8_t i = 0; i < NUM_INSTANTES; i++) {
        uint8_t idx = (uint8_t)round(
            INSTANTES[i] / 100.0f * (p.numMuestras - 1)
        );
        if (idx >= p.numMuestras) idx = p.numMuestras - 1;
        p.instantes[i] = p.muestras[idx];
    }
}

void enviarPisadaBLE(uint8_t idx) {
    Pisada& p = sesion[idx];
    for (uint8_t i = 0; i < NUM_INSTANTES; i++) {
        Muestra& m = p.instantes[i];
        uint8_t buf[26];
        memcpy(buf,      &m.velX1, 2);
        memcpy(buf + 2,  &m.velY1, 2);
        memcpy(buf + 4,  &m.velZ1, 2);
        memcpy(buf + 6,  &m.posX1, 2);
        memcpy(buf + 8,  &m.posY1, 2);
        memcpy(buf + 10, &m.posZ1, 2);
        memcpy(buf + 12, &m.velX2, 2);
        memcpy(buf + 14, &m.velY2, 2);
        memcpy(buf + 16, &m.velZ2, 2);
        memcpy(buf + 18, &m.posX2, 2);
        memcpy(buf + 20, &m.posY2, 2);
        memcpy(buf + 22, &m.posZ2, 2);
        memcpy(buf + 24, &m.t_ms,  2);
        pisadaChar.writeValue(buf, 26);
        delay(30);
    }
}

void mostrarPisadaSerial(uint8_t idx) {
    Pisada& p = sesion[idx];
    Serial.println();
    Serial.print("--- PISADA "); Serial.print(idx + 1);
    Serial.print(" | duracion: "); Serial.print(p.duracion_ms);
    Serial.print(" ms | instantes: "); Serial.println(NUM_INSTANTES);
    Serial.println("% inst | t_ms | vX1    vY1    vZ1   | roll1   pitch1  yaw1  | vX2    vY2    vZ2   | roll2   pitch2  yaw2");
    Serial.println("-------+------+---------------------+-----------------------+---------------------+-----------------------");

    for (uint8_t i = 0; i < NUM_INSTANTES; i++) {
        Muestra& m = p.instantes[i];
        Serial.print(INSTANTES[i], 0); Serial.print("%  | ");
        Serial.print(m.t_ms); Serial.print(" | ");
        Serial.print(m.velX1 / 100.0f, 2); Serial.print(" ");
        Serial.print(m.velY1 / 100.0f, 2); Serial.print(" ");
        Serial.print(m.velZ1 / 100.0f, 2); Serial.print(" | ");
        Serial.print(m.posX1 / 100.0f, 2); Serial.print(" ");
        Serial.print(m.posY1 / 100.0f, 2); Serial.print(" ");
        Serial.print(m.posZ1 / 100.0f, 2); Serial.print(" | ");
        Serial.print(m.velX2 / 100.0f, 2); Serial.print(" ");
        Serial.print(m.velY2 / 100.0f, 2); Serial.print(" ");
        Serial.print(m.velZ2 / 100.0f, 2); Serial.print(" | ");
        Serial.print(m.posX2 / 100.0f, 2); Serial.print(" ");
        Serial.print(m.posY2 / 100.0f, 2); Serial.print(" ");
        Serial.println(m.posZ2 / 100.0f, 2);
    }
}

// ============================================================
// FUNCION DE CALIBRACION ESTATICA
// ============================================================
void calibrarPosicionNeutra() {
    Serial.println();
    Serial.println("========================================");
    Serial.println("CALIBRACION — Mantenga el pie QUIETO");
    Serial.println("durante 5 segundos...");
    Serial.println("========================================");

    // Reiniciar filtros Madgwick
    filtroAst.begin(FREQ_HZ);
    filtroMeta.begin(FREQ_HZ);

    // Dejar que los filtros converjan primero (2 segundos) 
    // empiezan en 0 y necesita dos segundos para tener los angulos exactos
    Serial.println("Convergiendo filtros...");
    uint32_t inicioConv = millis();
    while (millis() - inicioConv < 2000) {
        // Leer astragalo
        float ax1 = imuAstragalo.readFloatAccelX();
        float ay1 = imuAstragalo.readFloatAccelY();
        float az1 = imuAstragalo.readFloatAccelZ();
        float gx1 = imuAstragalo.readFloatGyroX();
        float gy1 = imuAstragalo.readFloatGyroY();
        float gz1 = imuAstragalo.readFloatGyroZ();
        filtroAst.updateIMU(gx1, gy1, gz1, ax1, ay1, az1);

        // Leer metatarsiano
        sensors_event_t accel, gyro, temp;
        imuMeta.getEvent(&accel, &gyro, &temp);
        float ax2 = accel.acceleration.x / 9.81f;
        float ay2 = accel.acceleration.y / 9.81f;
        float az2 = accel.acceleration.z / 9.81f;
        float gx2 = gyro.gyro.x * 180.0f / PI;
        float gy2 = gyro.gyro.y * 180.0f / PI;
        float gz2 = gyro.gyro.z * 180.0f / PI;
        filtroMeta.updateIMU(gx2, gy2, gz2, ax2, ay2, az2);

        delay(10);
    }

    // Promediar los siguientes 5 segundos
    Serial.println("Midiendo posicion neutra...");
    float sumRollAst=0, sumPitchAst=0, sumYawAst=0;
    float sumRollMeta=0, sumPitchMeta=0, sumYawMeta=0;
    uint16_t nMuestras = 0;

    //calibrar durante 5 s 
    uint32_t inicio = millis();
    while (millis() - inicio < TIEMPO_CALIBRACION_MS) {
        // Leer astragalo
        float ax1 = imuAstragalo.readFloatAccelX();
        float ay1 = imuAstragalo.readFloatAccelY();
        float az1 = imuAstragalo.readFloatAccelZ();
        float gx1 = imuAstragalo.readFloatGyroX();
        float gy1 = imuAstragalo.readFloatGyroY();
        float gz1 = imuAstragalo.readFloatGyroZ();
        filtroAst.updateIMU(gx1, gy1, gz1, ax1, ay1, az1);

        // Leer metatarsiano
        sensors_event_t accel, gyro, temp;
        imuMeta.getEvent(&accel, &gyro, &temp);
        float ax2 = accel.acceleration.x / 9.81f;
        float ay2 = accel.acceleration.y / 9.81f;
        float az2 = accel.acceleration.z / 9.81f;
        float gx2 = gyro.gyro.x * 180.0f / PI;
        float gy2 = gyro.gyro.y * 180.0f / PI;
        float gz2 = gyro.gyro.z * 180.0f / PI;
        filtroMeta.updateIMU(gx2, gy2, gz2, ax2, ay2, az2);

        // Acumular angulos
        sumRollAst  += filtroAst.getRoll();
        sumPitchAst += filtroAst.getPitch();
        sumYawAst   += filtroAst.getYaw();
        sumRollMeta  += filtroMeta.getRoll();
        sumPitchMeta += filtroMeta.getPitch();
        sumYawMeta   += filtroMeta.getYaw();

        nMuestras++;
        delay(10);
    }

    // Calcular medias = offsets
    rollOffAst  = sumRollAst  / nMuestras;
    pitchOffAst = sumPitchAst / nMuestras;
    yawOffAst   = sumYawAst   / nMuestras;
    rollOffMeta  = sumRollMeta  / nMuestras;
    pitchOffMeta = sumPitchMeta / nMuestras;
    yawOffMeta   = sumYawMeta   / nMuestras;

    Serial.println();
    Serial.println("========================================");
    Serial.println("CALIBRACION COMPLETADA");
    Serial.println("========================================");
    Serial.println("           ASTRAGALO  |  METATARSIANO");
    Serial.print(" Roll  off:  ");
    Serial.print(rollOffAst, 2); Serial.print("     |  ");
    Serial.println(rollOffMeta, 2);
    Serial.print(" Pitch off:  ");
    Serial.print(pitchOffAst, 2); Serial.print("     |  ");
    Serial.println(pitchOffMeta, 2);
    Serial.print(" Yaw   off:  ");
    Serial.print(yawOffAst, 2); Serial.print("     |  ");
    Serial.println(yawOffMeta, 2);
    Serial.println("========================================");
    Serial.println();

    // Enviar los offsets por BLE
    // Formato: [0xCA] + 6 int16 astragalo + 6 int16 meta = 13 bytes... no
    // Formato: [0xCA] + rollA + pitchA + yawA + rollM + pitchM + yawM
    //          1 byte + 6 * 2 bytes = 13 bytes
    // Pero usamos escalado x100 para 2 decimales de precision
    uint8_t bufCal[13];
    bufCal[0] = 0xCA;    // marcador de calibracion
    int16_t v;
    v = (int16_t)(rollOffAst  * 100.0f); memcpy(bufCal + 1,  &v, 2);
    v = (int16_t)(pitchOffAst * 100.0f); memcpy(bufCal + 3,  &v, 2);
    v = (int16_t)(yawOffAst   * 100.0f); memcpy(bufCal + 5,  &v, 2);
    v = (int16_t)(rollOffMeta * 100.0f); memcpy(bufCal + 7,  &v, 2);
    v = (int16_t)(pitchOffMeta * 100.0f); memcpy(bufCal + 9, &v, 2);
    v = (int16_t)(yawOffMeta   * 100.0f); memcpy(bufCal + 11, &v, 2);
    pisadaChar.writeValue(bufCal, 13);
    delay(50);

    calibrado = true;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    delay(3000);
    Serial.begin(115200);

    NRF_P1->PIN_CNF[8] = 1;
    NRF_P1->OUTSET = (1 << 8);
    delay(500);

    if (imuAstragalo.begin() != 0) {
        Serial.println("ERROR: IMU astragalo");
        while (1);
    }
    Serial.println("IMU astragalo OK");

    if (!imuMeta.begin_I2C(0x6B)) {
        Serial.println("ERROR: IMU metatarsiano");
        while (1);
    }
    Serial.println("IMU metatarsiano OK");

    filtroAst.begin(FREQ_HZ);
    filtroMeta.begin(FREQ_HZ);

    if (!BLE.begin()) {
        Serial.println("ERROR: BLE");
        while (1);
    }

    BLE.setLocalName("Tobillera");
    BLE.setAdvertisedService(datoService);
    datoService.addCharacteristic(pisadaChar);
    datoService.addCharacteristic(cmdChar);
    BLE.addService(datoService);
    BLE.advertise();

    Serial.println("Listo — esperando BLE...");
    Serial.println("Metodo AVGS + Calibracion estatica");
}

// ============================================================
// LOOP
// ============================================================
void loop() {

    //comprueba estar conectado a un disposituvo por ble 
    BLEDevice central = BLE.central();
    if (!central) return;

    Serial.print("Conectado: ");
    Serial.println(central.address());

    while (central.connected()) {

        // Comandos BLE
        if (cmdChar.written()) {
            uint8_t cmd = cmdChar.value()[0];

            if (cmd == 0x01 && !sesionActiva) {
                numPisadas = 0;

                // FASE 1: Calibracion estatica (5 segundos quieto)
                calibrarPosicionNeutra();

                // FASE 2: Empezar deteccion de pisadas
                sesionActiva = true;
                estado = ESPERANDO_MSW;
                //Serial.println("Sesion iniciada — puede empezar a correr");
                // se para auto en 10 segundos las proximas dos lineas:
                inicioSesionMs = millis();  // ← guardar tiempo de inicio
                Serial.println("Sesion iniciada — auto-stop en 10s");

            } else if (cmd == 0x02 && sesionActiva) {
                sesionActiva = false;
                estado = ESPERANDO_MSW;

              /* Se envían todas las pisadas:
                Serial.print("Enviando ");
                Serial.print(numPisadas);
                Serial.println(" pisadas...");

                for (uint8_t i = 0; i < numPisadas; i++) {
                    enviarPisadaBLE(i);
                    mostrarPisadaSerial(i);
                    delay(50);
                }*/
                // Descartar 2 primeras y 2 ultimas pisadas (aceleracion y frenado)
                uint8_t inicio = 2;
                uint8_t fin_idx = (numPisadas >= 2) ? (numPisadas - 2) : 0;
                uint8_t pisadasValidas = (fin_idx > inicio) ? (fin_idx - inicio) : 0;

                Serial.print("Total capturadas: "); Serial.println(numPisadas);
                Serial.print("Descartadas: 2 primeras + 2 ultimas");
                Serial.println();
                Serial.print("Enviando "); Serial.print(pisadasValidas);
                Serial.println(" pisadas validas...");

                for (uint8_t i = inicio; i < fin_idx; i++) {
                    enviarPisadaBLE(i);
                    mostrarPisadaSerial(i);
                    delay(50);
                }
                delay(100);
                uint8_t fin[1] = {0xFE};
                pisadaChar.writeValue(fin, 1);
                Serial.println("Listo");
            }
        }

        if (!sesionActiva) continue;
        // AUTO-STOP a los 10 segundos
        if (millis() - inicioSesionMs > 10000) {
            Serial.println();
            Serial.println("Auto-stop a los 10 segundos");
            sesionActiva = false;
            estado = ESPERANDO_MSW;

            /* Se envian todas las pisadas:
            Serial.print("Enviando ");
            Serial.print(numPisadas);
            Serial.println(" pisadas...");

            for (uint8_t i = 0; i < numPisadas; i++) {
                enviarPisadaBLE(i);
                mostrarPisadaSerial(i);
                delay(50);
            }*/
            // Descartar 2 primeras y 2 ultimas pisadas (aceleracion y frenado)
            uint8_t inicio = 2;
            uint8_t fin_idx = (numPisadas >= 2) ? (numPisadas - 2) : 0;
            uint8_t pisadasValidas = (fin_idx > inicio) ? (fin_idx - inicio) : 0;

            Serial.print("Total capturadas: "); Serial.println(numPisadas);
            Serial.print("Descartadas: 2 primeras + 2 ultimas");
            Serial.println();
            Serial.print("Enviando "); Serial.print(pisadasValidas);
            Serial.println(" pisadas validas...");

            for (uint8_t i = inicio; i < fin_idx; i++) {
                enviarPisadaBLE(i);
                mostrarPisadaSerial(i);
                delay(50);
            }

            delay(100);
            uint8_t fin[1] = {0xFE};
            pisadaChar.writeValue(fin, 1);
            Serial.println("Listo");
            continue;
        }

        // Control 100Hz
        static unsigned long ultimaMuestra = 0;
        // DECLARACIÓN de la variable anterior: debe ser static para que conserve su valor entre ciclos del loop
        static float gy1_Anterior = 0.0f;

        unsigned long ahora = micros();
        if (ahora - ultimaMuestra < INTERVALO_US) continue;
        ultimaMuestra = ahora;

        // Leer astragalo
        float ax1 = imuAstragalo.readFloatAccelX();
        float ay1 = imuAstragalo.readFloatAccelY();
        float az1 = imuAstragalo.readFloatAccelZ();
        float gx1 = imuAstragalo.readFloatGyroX();
        float gy1 = imuAstragalo.readFloatGyroY();
        float gz1 = imuAstragalo.readFloatGyroZ();

        // Leer metatarsiano
        sensors_event_t accel2, gyro2, temp2;
        imuMeta.getEvent(&accel2, &gyro2, &temp2);
        float ax2 = accel2.acceleration.x / 9.81f;
        float ay2 = accel2.acceleration.y / 9.81f;
        float az2 = accel2.acceleration.z / 9.81f;
        float gx2 = gyro2.gyro.x * 180.0f / PI;
        float gy2 = gyro2.gyro.y * 180.0f / PI;
        float gz2 = gyro2.gyro.z * 180.0f / PI;

        // ============================================================
        // METODO AVGS — Deteccion por giroscopio del astragalo
        // ============================================================
        

        switch (estado) {

            // ESPERANDO_MSW (Mid-Swing): El pie vuela hacia adelante. 
            // Buscamos el PICO MÁXIMO de rotación.
            case ESPERANDO_MSW:
                // Condición: La señal estaba por encima del umbral, pero el valor actual 
                // es menor que el anterior (es decir, acabamos de pasar la cima del pico).
                if (gy1_Anterior > UMBRAL_MSW_DPS && gy1 < gy1_Anterior 
                    && (millis() - finPisadaMs) > BLOQUEO_MS) {
                    estado = ESPERANDO_IC;
                }
                break;

            // ESPERANDO_IC (Initial Contact): El pie va a golpear el suelo.
            // Buscamos el CRUCE POR CERO de la velocidad angular.
            case ESPERANDO_IC:
                // Condición: La señal anterior era positiva (o 0) y la actual es negativa.
                // Esto marca el instante exacto en que la rotación se invierte por el impacto.
                if (gy1_Anterior >= 0.0f && gy1 < 0.0f && numPisadas < MAX_PISADAS) {
                    estado = EN_APOYO;
                    inicioPisada = millis();
                    sesion[numPisadas].numMuestras = 0;
                    resetEstado(estadoAst);
                    resetEstado(estadoMeta);
                    // IMPORTANTE: NO reiniciar Madgwick aqui,
                    // sino se perderia la calibracion
                    Serial.print("Pisada "); Serial.println(numPisadas + 1);
                }
                break;

            // EN_APOYO: El pie está en el suelo.
            // Buscamos el TC (Terminal Contact / Despegue)
            case EN_APOYO:
                // Actualizar con offsets de calibracion aplicados
                actualizarEstado(estadoAst,  filtroAst,
                                 ax1, ay1, az1, gx1, gy1, gz1,
                                 rollOffAst, pitchOffAst, yawOffAst);
                actualizarEstado(estadoMeta, filtroMeta,
                                 ax2, ay2, az2, gx2, gy2, gz2,
                                 rollOffMeta, pitchOffMeta, yawOffMeta);

                {
                    Pisada& p = sesion[numPisadas];
                    if (p.numMuestras < MAX_MUESTRAS) {
                        Muestra& m = p.muestras[p.numMuestras++];
                        m.velX1 = toInt16Vel(estadoAst.velX);
                        m.velY1 = toInt16Vel(estadoAst.velY);
                        m.velZ1 = toInt16Vel(estadoAst.velZ);
                        m.posX1 = toInt16Ang(estadoAst.roll);
                        m.posY1 = toInt16Ang(estadoAst.pitch);
                        m.posZ1 = toInt16Ang(estadoAst.yaw);
                        m.velX2 = toInt16Vel(estadoMeta.velX);
                        m.velY2 = toInt16Vel(estadoMeta.velY);
                        m.velZ2 = toInt16Vel(estadoMeta.velZ);
                        m.posX2 = toInt16Ang(estadoMeta.roll);
                        m.posY2 = toInt16Ang(estadoMeta.pitch);
                        m.posZ2 = toInt16Ang(estadoMeta.yaw);
                        m.t_ms  = (uint16_t)(millis() - inicioPisada);
                    }
                    
                    uint32_t duracion = millis() - inicioPisada;

                    // Detección del final de pisada (Toe-Off):
                    // El despegue genera una fuerte aceleración rotacional positiva de nuevo.
                    // Mantenemos tu idea del "cambio brusco", pero aplicada solo al eje Y.
                    bool cambioBrusco = (gy1 - gy1_Anterior) > 50.0f;

                    if (duracion > DURACION_MINIMA_MS &&
                        (cambioBrusco || duracion > DURACION_MAXIMA_MS)) {

                        if (p.numMuestras >= NUM_INSTANTES) {
                            p.duracion_ms = (uint16_t)duracion;
                            seleccionarInstantes(p);
                            numPisadas++;
                        }
                        estado = ESPERANDO_MSW;
                        finPisadaMs = millis();
                    }
                }
                break;

            default:
                estado = ESPERANDO_MSW;
                break;
        }

        // ============================================================
        // ACTUALIZAR VALORES ANTERIORES PARA EL SIGUIENTE CICLO
        // ============================================================
        gy1_Anterior = gy1;

        // Mantener filtros Madgwick actualizados incluso fuera del apoyo
        // (importante para no perder la referencia)
        if (estado != EN_APOYO) {
            filtroAst.updateIMU(gx1, gy1, gz1, ax1, ay1, az1);
            filtroMeta.updateIMU(gx2, gy2, gz2, ax2, ay2, az2);}
    }

    Serial.println("BLE desconectado");
}
