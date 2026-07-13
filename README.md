# TFG_repository
# Tobillera TFG — Análisis biomecánico de carrera

Sistema wearable para el análisis biomecánico de la pisada durante la carrera de velocidad, basado en dos IMUs colocadas en el astrágalo y el metatarsiano. Trabajo de Fin de Grado — Universidad Carlos III de Madrid (UC3M).

## Descripción

El dispositivo captura la velocidad lineal y la orientación angular del pie durante cada pisada a 100 Hz, detecta automáticamente los eventos de contacto inicial y despegue mediante el algoritmo AVGS (Angular Velocity-based Gait Segmentation), y transmite los datos por BLE a una aplicación web para su visualización y exportación.

## Hardware

- **Microcontrolador:** Seeed XIAO nRF52840 Sense Plus (BLE integrado)
- **IMU astrágalo:** LSM6DS3 integrada en el Xiao (I2C, dirección 0x6A)
- **IMU metatarsiano:** Adafruit LSM6DSOX (I2C, dirección 0x6B)
- **Alimentación:** batería LiPo 3.7V, 410 mAh

## Estructura del repositorio

```
tobillera-TFG/
├── README.md
├── firmware/
│   └── tobillera_avgs_calibrado/
│       └── tobillera_avgs_calibrado.ino
└── web/
    └── tobillera_visualizador.html
```

## Librerías necesarias

Instalar desde el Library Manager de Arduino IDE:

- `Seeed Arduino LSM6DS3` (v2.0.5)
- `Adafruit LSM6DS` 2.9.3
- `MadgwickAHRS`
- `ArduinoBLE`

Board package: `Seeed nRF52 mbed-enabled Boards` (v2.9.3) — seleccionar placa **Seeed XIAO nRF52840 Sense Plus**.

## Funcionamiento

1. **Calibración estática (5 s):** al iniciar la sesión el usuario permanece quieto para que el sistema registre la posición neutra del pie. Los ángulos se calculan como offsets relativos a esta referencia.

2. **Detección de pisadas (AVGS):** máquina de estados basada en la magnitud del giroscopio del astrágalo:
   - `ESPERANDO_MSW` → detecta el pico de mid-swing (>250°/s)
   - `ESPERANDO_IC` → detecta el contacto inicial (cruce descendente por 100°/s)
   - `EN_APOYO` → captura muestras a 100 Hz hasta detectar el despegue

3. **Selección de 9 instantes:** por cada pisada se extraen los valores biomecánicos en los porcentajes 0, 10, 20, 27, 35, 67, 92, 98 y 100 % del ciclo de apoyo.

4. **Transmisión BLE:** los datos se envían al terminar la sesión mediante paquetes de 26 bytes por instante más un marcador de fin.

## Uso

1. Subir el firmware al Xiao con Arduino IDE
2. Conectar la batería y colocar la tobillera
3. Abrir el archivo `tobillera_visualizador.html` en Google Chrome
4. Pulsar **Conectar BLE** y seleccionar "Tobillera"
5. Pulsar **Iniciar** y permanecer quieto 5 segundos
6. Correr los 100 m
7. Pulsar **Terminar** para recibir los datos
8. Exportar los resultados como CSV si se desea

## Referencias

- Ruiz-Ruiz et al. (2024) — *Improved running gait parameter estimation from single foot-mounted IMU data based on refined event detection*
- Luo et al. (2024) — *Refined gait event detection using zero-crossing methods*
- Fadillioglu et al. (2020) — *Validation of foot-mounted IMU gait event detection in overground running*

## Autora

Rosalía Carballo — Grado en Ingeniería Electrónica Industrial y Automática, UC3M

