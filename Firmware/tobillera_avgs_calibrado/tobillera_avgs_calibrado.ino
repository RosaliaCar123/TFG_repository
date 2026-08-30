
// ============================================================
// TOBILLERA TFG — AVGS + Calibracion estatica
//
// Flujo de uso (pensado para correr lejos del ordenador):
//   1. Se encienden los sensores (setup)
//   2. El ordenador manda CALIBRAR (0x01): se converge el filtro y se
//      promedian 5 segundos QUIETO -> offsets de posicion neutra
//   3. El micro NO empieza aun: espera una segunda señal INICIAR (0x03)
//   4. Tras INICIAR, la persona corre y puede alejarse todo lo que quiera:
//      la deteccion y el almacenamiento siguen aunque se pierda el BLE
//   5. Se almacenan los 9 instantes de cada pisada hasta detectar
//      PISADAS_TOTALES (20 por defecto): se descartan las 2 primeras,
//      las 2 ultimas, y las que se detectaron por TIMEOUT (no fiables)
//   6. Al llegar a PISADAS_TOTALES se DETIENE el almacenamiento (LED fijo)
//   7. La persona vuelve, el ordenador se reconecta y manda ENVIAR (0x02):
//      se transmiten las pisadas validas restantes por BLE
//   8. Los angulos guardados son RELATIVOS a la posicion neutra
//
// Deteccion por giroscopio (metodo AVGS)
//  Angular Velocity-based Gait Segmentation
// Se divide en tres fases la pisada:
// 1. vuelo: pico grande de rotacion: hacia delante
// 2. Contacto: la señal cae bruscamente cuando el pie impacta en el suelo
// 3. Apoyo: señal estable cerca de cero
// 4. Despegue: el pie vuelve a rotar- la velocidad sube
//
// Deteccion afinada (Ruiz-Ruiz et al.): en vez del modulo del giroscopio
// se usa unicamente gy1 (eje Y del astragalo), con deteccion de pico local
// para el MSW, cruce por cero + pico de aceleracion (fusion) para el IC,
// e inflexion (minimo local seguido de subida) para el despegue

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
#define FREQ_HZ          200
#define INTERVALO_US     5000
#define NUM_INSTANTES    9
#define UMBRAL_RUIDO     0.04f

// Nº de pisadas a almacenar antes de parar. Se descartan las 2 primeras
// (aceleracion), las 2 ultimas (frenado) y las que se detectaron por TIMEOUT
// (deteccion no fiable) -> con margen extra para que sigan sobrando
// suficientes validas aunque se descarten algunas por TIMEOUT.
#define PISADAS_TOTALES  20

// El array de pisadas nunca necesita ser mayor que PISADAS_TOTALES (la
// grabacion se para en cuanto se alcanza ese numero) -> +2 de margen de
// seguridad para el chequeo defensivo de numPisadas < MAX_PISADAS.
#define MAX_PISADAS      (PISADAS_TOTALES + 2)

// A 200Hz, para cubrir toda la duracion maxima de una pisada
// (DURACION_MAXIMA_MS = 500ms) hacen falta 100 muestras -> margen hasta 110.
#define MAX_MUESTRAS     110

// Radio de la ventana (en muestras a cada lado) que se promedia al elegir
// cada uno de los 9 instantes de interes, en vez de coger solo la muestra
// mas cercana al %: a 200Hz, +-2 muestras = +-10ms de suavizado.
#define VENTANA_INSTANTE 0

// --- Comandos que manda el ordenador por BLE ---
#define CMD_CALIBRAR  0x01  // converge filtros + 5s quieto -> offsets
#define CMD_INICIAR   0x03  // arranca la deteccion/almacenamiento (a correr)
#define CMD_ENVIAR    0x02  // transmite las pisadas validas almacenadas

// --- Marcador del paquete de diagnostico (por pisada, 11 bytes) ---
// Se guarda con cada pisada capturada (buena o descartada) y se manda junto
// con el resto de la sesion al reconectar tras la carrera -> permite ver
// hueco/causa/pico/valle de una carrera real sin depender de llevar nada
// conectado durante la carrera (ni cable ni movil).
#define MARCADOR_DIAG 0xD1

// --- Deteccion por giroscopio (metodo AVGS, eje Y unicamente) ---
#define UMBRAL_MSW_DPS       190.0f // altura minima del pico de gy1 durante el vuelo
#define DURACION_MINIMA_MS   150// descarta pisadas inferiores a 150ms: 6.67 Hz de zancada
#define DURACION_MAXIMA_MS   500
#define BLOQUEO_MS           100 //evita oscilaciones 100ms despues de una pisada (filtra)

#define UMBRAL_DESPEGUE_DPS  80.0f // subida de gy1 desde su minimo en apoyo que marca el despegue

// --- Proteccion del filtro Madgwick contra impactos/dinamica fuerte ---
// El acelerometro solo sirve como referencia de "hacia donde tira la
// gravedad" cuando su magnitud esta cerca de 1g. Fuera de esta banda
// (impacto contra el suelo, dinamica fuerte del balanceo) esa lectura
// esta contaminada y NO se usa para corregir el angulo (ver actualizarFiltro).
#define UMBRAL_ACEL_MIN_G  0.7f
#define UMBRAL_ACEL_MAX_G  1.3f

// --- Envio BLE ---
#define BLE_DELAY_MS 120 // margen entre notificaciones para que el central no pierda paquetes

// --- Calibracion estatica ---
#define TIEMPO_CALIBRACION_MS   5000    // 5 segundos quieto
#define MUESTRAS_CALIBRACION    500     // ritmo fijo de 10ms/muestra (no depende de FREQ_HZ) -> 500 muestras = 5s

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
    Muestra muestras[MAX_MUESTRAS]; //a 200hz
    Muestra instantes[NUM_INSTANTES]; //los 9 instantes
    uint8_t  numMuestras;
    uint16_t duracion_ms;
    bool     porTimeout; // true = el despegue NO se detecto por inflexion real
                         // (se salvo por el timeout de seguridad): deteccion
                         // no fiable, se descarta al enviar
    // --- Diagnostico (se manda para TODAS las pisadas, incluso descartadas) ---
    float    diagPicoMSW;
    float    diagGy1Min;
    uint32_t diagHueco;
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
bool     calibrado    = false;
uint32_t ultimaMuestra = 0; // reloj del muestreo a FREQ_HZ (global: el loop ya no es bloqueante)

// --- Maquina de estados de la aplicacion (nivel superior) ---
// Independiente del BLE: la deteccion sigue aunque se pierda la conexion.
enum EstadoApp {
    APP_IDLE,             // recien encendido / esperando comando de CALIBRAR
    APP_ESPERANDO_START,  // calibrado, filtros calientes, esperando comando de INICIAR
    APP_GRABANDO,         // detectando y almacenando pisadas (sin depender del BLE)
    APP_COMPLETO          // PISADAS_TOTALES guardadas, esperando reconexion para ENVIAR
};
EstadoApp estadoApp = APP_IDLE;

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
float gy1Anterior = 0;

// --- Estado auxiliar para la deteccion afinada en gy1 ---
bool     ascendiendoMSW  = false; // gy1 esta subiendo -> buscando el pico del vuelo
bool     descendiendoApoyo = false; // gy1 esta bajando dentro del apoyo -> buscando el minimo
float    gy1Min          = 0;     // minimo local de gy1 durante el apoyo (para detectar el despegue)
float    mswPico         = 0;     // valor del ultimo pico de MSW detectado (diagnostico)
uint32_t huecoPisada     = 0;     // ms desde que acabo la pisada anterior (diagnostico)

// LIMITACION CONOCIDA (gimbal lock): roll/pitch/yaw son fiables mientras
// pitch se mantenga lejos de +/-90 grados. Cuando el pie rota mucho en el
// tramo de despegue (pitch > ~55-60 grados, tipicamente hacia el 67-100%
// de la pisada), roll y yaw pueden dar saltos grandes sin significado fisico
// aunque esten dentro de wrap180 -> ver conversacion del TFG sobre esto.
// Solucion de fondo pendiente: transmitir cuaterniones en vez de angulos de Euler.
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

// Envuelve un angulo al rango [-180, 180]. Necesario porque el angulo
// RELATIVO (absoluto - offset) puede cruzar la frontera de +/-180 y saltar
// de golpe (p.ej. de +179 a -179 con un movimiento real de solo 2 grados).
// Esto expresa siempre la rotacion como el camino mas corto respecto a la
// posicion neutra, y elimina esos saltos falsos en Roll y Yaw.
inline float wrap180(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// Actualiza un filtro Madgwick protegiendo la correccion del acelerometro
// contra impactos/dinamica fuerte. El acelerometro solo es una referencia
// valida de "hacia donde tira la gravedad" cuando su magnitud ronda 1g; en
// un impacto (varios g) o durante un balanceo muy dinamico esa lectura esta
// contaminada por aceleracion lineal y corregiria el angulo hacia un valor
// erroneo. Fuera de la banda [UMBRAL_ACEL_MIN_G, UMBRAL_ACEL_MAX_G] se pasa
// (0,0,0): la libreria Madgwick trata ese caso como "sin acelerometro" y
// esa muestra se integra SOLO con el giroscopio, que no se ve afectado por
// el golpe.
void actualizarFiltro(Madgwick& filtro, float gx, float gy, float gz,
                       float ax, float ay, float az) {
    float accMag = sqrt(ax*ax + ay*ay + az*az);
    if (accMag > UMBRAL_ACEL_MIN_G && accMag < UMBRAL_ACEL_MAX_G) {
        filtro.updateIMU(gx, gy, gz, ax, ay, az);
    } else {
        filtro.updateIMU(gx, gy, gz, 0.0f, 0.0f, 0.0f);
    }
}

void actualizarEstado(EstadoIMU& e, Madgwick& filtro,
                      float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float rollOff, float pitchOff, float yawOff) {

    unsigned long ahora = micros();
    float dt = (ahora - e.tiempoAnterior) / 1000000.0f;
    e.tiempoAnterior = ahora;
    if (dt <= 0 || dt > 0.1f) return;

    actualizarFiltro(filtro, gx, gy, gz, ax, ay, az);
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

    // Angulos RELATIVOS: restar el offset de calibracion y envolver a [-180,180]
    // (wrap180 evita los saltos falsos cerca de la frontera de +/-180 grados)
    e.roll  = wrap180(filtro.getRoll()  - rollOff);
    e.pitch = wrap180(filtro.getPitch() - pitchOff);
    e.yaw   = wrap180(filtro.getYaw()   - yawOff);
}

// aproxiar a dos decimales para reducir a 2bytes: 34767 son limites 
inline int16_t toInt16Vel(float v) { return (int16_t)constrain(v * 100.0f, -32767, 32767); }
inline int16_t toInt16Ang(float a) { return (int16_t)constrain(a * 100.0f, -32767, 32767); }

// Media circular de un conjunto de angulos (grados). Promediar angulos con
// una media normal falla cerca de la frontera +-180: p.ej. 179 y -179 estan
// a solo 2 grados de distancia real, pero una media aritmetica a lo bruto
// da 0 (el lado opuesto). Pasando por seno/coseno se evita ese salto falso.
float mediaCircular(const float* valoresGrados, int n) {
    float sumSin = 0, sumCos = 0;
    for (int i = 0; i < n; i++) {
        float rad = valoresGrados[i] * PI / 180.0f;
        sumSin += sin(rad);
        sumCos += cos(rad);
    }
    return atan2(sumSin, sumCos) * 180.0f / PI;
}

// Para cada uno de los 9 instantes de interes, en vez de coger solo la
// muestra mas cercana al %, se promedia una pequeña ventana de muestras
// alrededor (+-VENTANA_INSTANTE) para suavizar el ruido de una unica
// lectura puntual. Las velocidades se promedian de forma normal; los
// angulos (roll/pitch/yaw) necesitan media circular (ver mediaCircular).
void seleccionarInstantes(Pisada& p) {
    const int ANCHO = 2 * VENTANA_INSTANTE + 1;
    for (uint8_t i = 0; i < NUM_INSTANTES; i++) {
        int idxCentral = (int)round(INSTANTES[i] / 100.0f * (p.numMuestras - 1));
        if (idxCentral >= p.numMuestras) idxCentral = p.numMuestras - 1;
        if (idxCentral < 0) idxCentral = 0;

        int idxIni = idxCentral - VENTANA_INSTANTE;
        int idxFin = idxCentral + VENTANA_INSTANTE;
        if (idxIni < 0) idxIni = 0;
        if (idxFin >= p.numMuestras) idxFin = p.numMuestras - 1;
        int n = idxFin - idxIni + 1;

        int32_t sVelX1=0, sVelY1=0, sVelZ1=0, sVelX2=0, sVelY2=0, sVelZ2=0;
        int32_t sT = 0;
        float rollAst[ANCHO], pitchAst[ANCHO], yawAst[ANCHO];
        float rollMeta[ANCHO], pitchMeta[ANCHO], yawMeta[ANCHO];

        int k = 0;
        for (int j = idxIni; j <= idxFin; j++, k++) {
            Muestra& m = p.muestras[j];
            sVelX1 += m.velX1; sVelY1 += m.velY1; sVelZ1 += m.velZ1;
            sVelX2 += m.velX2; sVelY2 += m.velY2; sVelZ2 += m.velZ2;
            sT += m.t_ms;
            rollAst[k]  = m.posX1 / 100.0f;
            pitchAst[k] = m.posY1 / 100.0f;
            yawAst[k]   = m.posZ1 / 100.0f;
            rollMeta[k]  = m.posX2 / 100.0f;
            pitchMeta[k] = m.posY2 / 100.0f;
            yawMeta[k]   = m.posZ2 / 100.0f;
        }

        Muestra& out = p.instantes[i];
        out.velX1 = (int16_t)(sVelX1 / n); out.velY1 = (int16_t)(sVelY1 / n); out.velZ1 = (int16_t)(sVelZ1 / n);
        out.velX2 = (int16_t)(sVelX2 / n); out.velY2 = (int16_t)(sVelY2 / n); out.velZ2 = (int16_t)(sVelZ2 / n);
        out.t_ms  = (uint16_t)(sT / n);

        out.posX1 = toInt16Ang(mediaCircular(rollAst, n));
        out.posY1 = toInt16Ang(mediaCircular(pitchAst, n));
        out.posZ1 = toInt16Ang(mediaCircular(yawAst, n));
        out.posX2 = toInt16Ang(mediaCircular(rollMeta, n));
        out.posY2 = toInt16Ang(mediaCircular(pitchMeta, n));
        out.posZ2 = toInt16Ang(mediaCircular(yawMeta, n));
    }
}

// ============================================================
// DIAGNOSTICO POR BLE (11 bytes) de una pisada ya almacenada.
// Se manda junto con el resto de la sesion al reconectar tras la
// carrera — no depende de llevar nada conectado durante la carrera,
// funciona igual con o sin movil (y con cualquier movil, iOS incluido).
// Formato: [0xD1][pisada][causa 0/1][duracion u16][picoMSW i16 x10]
//          [gy1Min i16 x10][hueco u16]
// NOTA: aqui se usa x10 (no x100 como en las muestras normales) porque
// mswPico y gy1Min pueden superar 600 dps, y a escala x100 desbordarian
// un int16 (max 327.67) dando valores absurdos/con signo invertido.
// ============================================================
void enviarDiagnosticoBLE(uint8_t idx) {
    Pisada& p = sesion[idx];

    uint8_t buf[11];
    buf[0] = MARCADOR_DIAG;
    buf[1] = idx + 1;
    buf[2] = p.porTimeout ? 1 : 0;
    memcpy(buf + 3, &p.duracion_ms, 2);
    int16_t v;
    v = (int16_t)constrain(p.diagPicoMSW * 10.0f, -32767, 32767); memcpy(buf + 5, &v, 2);
    v = (int16_t)constrain(p.diagGy1Min  * 10.0f, -32767, 32767); memcpy(buf + 7, &v, 2);
    uint16_t huecoAcotado = (p.diagHueco > 60000) ? 60000 : (uint16_t)p.diagHueco;
    memcpy(buf + 9, &huecoAcotado, 2);

    pisadaChar.writeValue(buf, 11);
}

void enviarPisadaBLE(uint8_t idx) {
    Pisada& p = sesion[idx];
    for (uint8_t i = 0; i < NUM_INSTANTES; i++) {
        Muestra& m = p.instantes[i];
        uint8_t buf[27];
        buf[0] = i; // indice del instante (0-8): permite a la app detectar paquetes BLE perdidos
        memcpy(buf + 1,  &m.velX1, 2);
        memcpy(buf + 3,  &m.velY1, 2);
        memcpy(buf + 5,  &m.velZ1, 2);
        memcpy(buf + 7,  &m.posX1, 2);
        memcpy(buf + 9,  &m.posY1, 2);
        memcpy(buf + 11, &m.posZ1, 2);
        memcpy(buf + 13, &m.velX2, 2);
        memcpy(buf + 15, &m.velY2, 2);
        memcpy(buf + 17, &m.velZ2, 2);
        memcpy(buf + 19, &m.posX2, 2);
        memcpy(buf + 21, &m.posY2, 2);
        memcpy(buf + 23, &m.posZ2, 2);
        memcpy(buf + 25, &m.t_ms,  2);
        pisadaChar.writeValue(buf, 27);
        delay(BLE_DELAY_MS);
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
// FUNCION DE CALIBRACION ESTATICA converger el filtro+ calibracion
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
        actualizarFiltro(filtroAst, gx1, gy1, gz1, ax1, ay1, az1);

        // Leer metatarsiano
        sensors_event_t accel, gyro, temp;
        imuMeta.getEvent(&accel, &gyro, &temp);
        float ax2 = accel.acceleration.x / 9.81f;
        float ay2 = accel.acceleration.y / 9.81f;
        float az2 = accel.acceleration.z / 9.81f;
        float gx2 = gyro.gyro.x * 180.0f / PI;
        float gy2 = gyro.gyro.y * 180.0f / PI;
        float gz2 = gyro.gyro.z * 180.0f / PI;
        actualizarFiltro(filtroMeta, gx2, gy2, gz2, ax2, ay2, az2);

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
        actualizarFiltro(filtroAst, gx1, gy1, gz1, ax1, ay1, az1);

        // Leer metatarsiano
        sensors_event_t accel, gyro, temp;
        imuMeta.getEvent(&accel, &gyro, &temp);
        float ax2 = accel.acceleration.x / 9.81f;
        float ay2 = accel.acceleration.y / 9.81f;
        float az2 = accel.acceleration.z / 9.81f;
        float gx2 = gyro.gyro.x * 180.0f / PI;
        float gy2 = gyro.gyro.y * 180.0f / PI;
        float gz2 = gyro.gyro.z * 180.0f / PI;
        actualizarFiltro(filtroMeta, gx2, gy2, gz2, ax2, ay2, az2);

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
    delay(BLE_DELAY_MS * 2);

    calibrado = true;
}

// ============================================================
// ENVIO DE LA SESION ALMACENADA
// Descarta 2 primeras + 2 ultimas (aceleracion/frenado) y ademas las
// que se detectaron por TIMEOUT (deteccion no fiable, ver porTimeout).
// Las "raras" pero detectadas por INFLEXION SI se envian: pueden ser
// variabilidad real, no ruido, y eso se decide en el analisis, no aqui.
// ============================================================
void enviarSesionBLE() {
    uint8_t inicio = 2;
    uint8_t fin_idx = (numPisadas >= 2) ? (numPisadas - 2) : 0;

    uint8_t pisadasValidas = 0;
    uint8_t descartadasTimeout = 0;
    for (uint8_t i = inicio; i < fin_idx; i++) {
        if (sesion[i].porTimeout) descartadasTimeout++;
        else pisadasValidas++;
    }

    Serial.print("Total capturadas: "); Serial.println(numPisadas);
    Serial.print("Descartadas: 2 primeras + 2 ultimas + ");
    Serial.print(descartadasTimeout); Serial.println(" por TIMEOUT");
    Serial.print("Enviando "); Serial.print(pisadasValidas);
    Serial.println(" pisadas validas...");

    // Diagnostico de TODAS las capturadas (incluidas las descartadas), para
    // poder revisar hueco/causa/pico/valle de la sesion completa sin haber
    // necesitado nada conectado durante la carrera
    for (uint8_t i = 0; i < numPisadas; i++) {
        enviarDiagnosticoBLE(i);
        delay(BLE_DELAY_MS);
    }
    delay(BLE_DELAY_MS);

    for (uint8_t i = inicio; i < fin_idx; i++) {
        if (sesion[i].porTimeout) continue; // deteccion no fiable, se descarta
        enviarPisadaBLE(i);
        mostrarPisadaSerial(i);
        delay(BLE_DELAY_MS);
    }
    delay(BLE_DELAY_MS * 2);
    uint8_t fin[1] = {0xFE};
    pisadaChar.writeValue(fin, 1);
    delay(BLE_DELAY_MS);
    Serial.println("Listo");
}

// ============================================================
// MUESTREO A FREQ_HZ + DETECCION AVGS
// Se llama fuera del bucle de conexion: la deteccion sigue aunque
// se pierda el BLE. Si no estamos grabando, solo mantiene los
// filtros Madgwick calientes para no perder la referencia.
// ============================================================
void muestrear() {
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

    // Fuera de grabacion: solo mantener los filtros al dia
    if (estadoApp != APP_GRABANDO) {
        actualizarFiltro(filtroAst, gx1, gy1, gz1, ax1, ay1, az1);
        actualizarFiltro(filtroMeta, gx2, gy2, gz2, ax2, ay2, az2);
        return;
    }

    // ============================================================
    // METODO AVGS — Deteccion por giroscopio del astragalo (solo gy1)
    // ============================================================
    switch (estado) {

        // Buscando el balanceo (MSW): pico local maximo de gy1.
        case ESPERANDO_MSW:
            if (gy1 > gy1Anterior) {
                ascendiendoMSW = true;
            } else {
                if (ascendiendoMSW && gy1Anterior > UMBRAL_MSW_DPS
                    && (millis() - finPisadaMs) > BLOQUEO_MS) {
                    estado = ESPERANDO_IC;
                    mswPico = gy1Anterior; // diagnostico: altura real del pico
                }
                ascendiendoMSW = false;
            }
            break;

        // Detectando el impacto (IC): cinematica pura.
        // gy1 cruza por cero de positivo a negativo: el pie deja de rotar
        // hacia delante porque toca el suelo.
        case ESPERANDO_IC:
            if (gy1Anterior > 0 && gy1 <= 0 && numPisadas < MAX_PISADAS) {
                estado = EN_APOYO;
                inicioPisada = millis();
                sesion[numPisadas].numMuestras = 0;
                resetEstado(estadoAst);
                resetEstado(estadoMeta);
                descendiendoApoyo = false;
                gy1Min = gy1;
                huecoPisada = inicioPisada - finPisadaMs; // diagnostico
                // IMPORTANTE: NO reiniciar Madgwick aqui, se perderia la calibracion
                // Diagnostico: si el hueco desde la pisada anterior es muy corto
                // (cerca de BLOQUEO_MS), sospechoso de ser la cola de esa misma
                // pisada en vez de un paso nuevo de verdad
                Serial.print("Pisada "); Serial.print(numPisadas + 1);
                Serial.print(" | pico MSW="); Serial.print(mswPico);
                Serial.print(" dps | hueco desde anterior=");
                Serial.print(huecoPisada);
                Serial.println("ms");
            }
            break;

        case EN_APOYO:
            // Calcula y guarda cada 10 ms la velocidad y los angulos
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
                // Despegue (Terminal Contact): inflexion de gy1 (minimo local
                // seguido de subida), o timeout de seguridad
                uint32_t duracion = millis() - inicioPisada;

                if (gy1 < gy1Min) {
                    gy1Min = gy1;
                    descendiendoApoyo = true;
                }
                bool inflexionDespegue = descendiendoApoyo
                                          && (gy1 - gy1Min) > UMBRAL_DESPEGUE_DPS;

                if (duracion > DURACION_MINIMA_MS &&
                    (inflexionDespegue || duracion > DURACION_MAXIMA_MS)) {

                    // Diagnostico: saber SI el despegue se detecto por inflexion real
                    // de gy1 (fiable) o si tuvo que salvarse por el timeout de
                    // seguridad (indicio de que el umbral no encaja con este ritmo),
                    // y que tan profundo fue el valle antes de disparar (un valle muy
                    // superficial es indicio de un despegue prematuro/falso)
                    Serial.print("  -> fin pisada: duracion="); Serial.print(duracion);
                    Serial.print("ms | causa=");
                    Serial.print(inflexionDespegue ? "INFLEXION" : "TIMEOUT (revisar UMBRAL_DESPEGUE_DPS)");
                    Serial.print(" | valle gy1Min="); Serial.print(gy1Min);
                    Serial.println(" dps");

                    if (p.numMuestras >= NUM_INSTANTES) {
                        p.duracion_ms  = (uint16_t)duracion;
                        p.porTimeout   = !inflexionDespegue;
                        // Guardar el diagnostico junto con la pisada: se manda
                        // por BLE con el resto al reconectar (no hace falta
                        // llevar nada conectado durante la carrera)
                        p.diagPicoMSW  = mswPico;
                        p.diagGy1Min   = gy1Min;
                        p.diagHueco    = huecoPisada;
                        seleccionarInstantes(p);
                        numPisadas++;
                    }
                    estado = ESPERANDO_MSW;
                    finPisadaMs = millis();

                    // ¿Ya tenemos el objetivo? -> parar de almacenar
                    if (numPisadas >= PISADAS_TOTALES) {
                        estadoApp = APP_COMPLETO;
                        Serial.println();
                        Serial.print(PISADAS_TOTALES);
                        Serial.println(" pisadas almacenadas — vuelve al ordenador para descargar");
                    }
                }
            }
            break;

        default:
            estado = ESPERANDO_MSW;
            break;
    }

    gy1Anterior = gy1;

    // Mantener filtros Madgwick actualizados fuera del apoyo (no perder referencia)
    if (estado != EN_APOYO) {
        actualizarFiltro(filtroAst, gx1, gy1, gz1, ax1, ay1, az1);
        actualizarFiltro(filtroMeta, gx2, gy2, gz2, ax2, ay2, az2);
    }
}

// ============================================================
// LED integrado — indica el estado sin necesidad del ordenador
// (XIAO nRF52840: LED activo en BAJO, LOW = encendido)
// ============================================================
void actualizarLED() {
    switch (estadoApp) {
        case APP_ESPERANDO_START:
            // parpadeo lento: calibrado, esperando la señal de inicio
            digitalWrite(LED_BUILTIN, (millis() / 500) % 2 ? HIGH : LOW);
            break;
        case APP_GRABANDO:
            // parpadeo rapido: grabando, corre
            digitalWrite(LED_BUILTIN, (millis() / 150) % 2 ? HIGH : LOW);
            break;
        case APP_COMPLETO:
            digitalWrite(LED_BUILTIN, LOW);  // fijo encendido: ya puedes volver
            break;
        default:
            digitalWrite(LED_BUILTIN, HIGH); // apagado
            break;
    }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    delay(3000);
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // apagado (activo en BAJO)

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

    // Al desconectarse (p.ej. la persona se aleja corriendo) hay que volver a
    // anunciar para que el ordenador pueda reconectar al terminar la carrera
    BLE.setEventHandler(BLEDisconnected, onBLEDisconnect);

    BLE.advertise();

    Serial.println("Listo — esperando BLE...");
    Serial.println("Metodo AVGS + Calibracion estatica");
}

// ============================================================
// Handler de desconexion BLE: vuelve a anunciar para permitir
// que el ordenador reconecte cuando la persona regrese
// ============================================================
void onBLEDisconnect(BLEDevice central) {
    Serial.println("BLE desconectado — re-anunciando");
    BLE.advertise();
}

// ============================================================
// LOOP — no bloqueante
// La deteccion/almacenamiento corre SIEMPRE que estemos grabando,
// haya o no conexion BLE (la persona puede alejarse corriendo).
// ============================================================
void loop() {

    // Procesa la pila BLE (conexiones, comandos). No bloquea: devuelve
    // el central si hay alguien conectado, o nada si no.
    BLEDevice central = BLE.central();

    // --- Comandos del ordenador (solo cuando hay conexion) ---
    if (central && central.connected() && cmdChar.written()) {
        uint8_t cmd = cmdChar.value()[0];

        // CALIBRAR: converge filtros + 5s quieto + envia offsets
        if (cmd == CMD_CALIBRAR && estadoApp == APP_IDLE) {
            numPisadas = 0;
            calibrarPosicionNeutra();
            estado = ESPERANDO_MSW;
            estadoApp = APP_ESPERANDO_START;
            ultimaMuestra = micros();  // arrancar el reloj de muestreo
            Serial.println("Calibrado — esperando señal de INICIO (0x03) para correr");
        }
        // INICIAR: arranca la deteccion. La persona ya puede alejarse
        else if (cmd == CMD_INICIAR && estadoApp == APP_ESPERANDO_START) {
            numPisadas = 0;
            estado = ESPERANDO_MSW;
            estadoApp = APP_GRABANDO;
            Serial.print("GRABANDO — corre. Para sola tras "); Serial.print(PISADAS_TOTALES);
            Serial.println(" pisadas.");
        }
        // ENVIAR: transmite las pisadas validas almacenadas
        else if (cmd == CMD_ENVIAR && numPisadas > 0) {
            Serial.println("Enviando sesion almacenada...");
            enviarSesionBLE();
            estadoApp = APP_IDLE;  // listo para otra sesion (recalibrar)
        }
    }

    // --- Muestreo a FREQ_HZ + deteccion (independiente del BLE) ---
    // Se muestrea tambien en ESPERANDO_START para mantener los filtros
    // Madgwick calientes hasta que llegue la señal de inicio.
    if (estadoApp == APP_ESPERANDO_START || estadoApp == APP_GRABANDO) {
        unsigned long ahora = micros();
        if (ahora - ultimaMuestra >= INTERVALO_US) {
            ultimaMuestra = ahora;
            muestrear();
        }
    }

    // --- Indicacion por LED (para saber el estado sin el ordenador) ---
    actualizarLED();
}