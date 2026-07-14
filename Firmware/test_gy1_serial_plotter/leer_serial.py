"""
Vuelca el buffer del sketch test_gy1_serial_plotter a un CSV limpio,
sin depender de copiar/pegar desde el Monitor Serie de Arduino.

Requisitos:
    pip install pyserial

Uso:
    1. Cierra el Monitor Serie del Arduino IDE (el puerto solo lo puede
       usar un programa a la vez).
    2. Ajusta PUERTO si no es COM7.
    3. Ejecuta: python leer_serial.py
    4. Abre captura.csv directamente con Excel (doble clic).
"""

import serial
import time

PUERTO = "COM7"
BAUDIOS = 115200
ARCHIVO_SALIDA = "captura.csv"

ser = serial.Serial(PUERTO, BAUDIOS, timeout=1)
time.sleep(2)  # dar tiempo a que el puerto se estabilice

ser.reset_input_buffer()
ser.write(b'd')

print("Esperando datos... (Ctrl+C para cancelar)")
with open(ARCHIVO_SALIDA, "w", encoding="utf-8") as f:
    while True:
        linea = ser.readline().decode(errors="ignore").strip()
        if not linea:
            continue
        print(linea)
        if linea.startswith("FIN"):
            break
        f.write(linea + "\n")

ser.close()
print(f"\nGuardado en {ARCHIVO_SALIDA}")
