import bluetooth
import time
from machine import Pin, PWM

# ============================================================
# RECEPTOR BLE DEL PROTOTIPO A ESCALA
# ============================================================
# Este programa se ejecuta en el ESP32-S3 del robot diferencial.
# Su función es recibir por BLE la consigna calculada por el guante
# y convertirla en señales PWM para el driver L298N.
#
# Importante: este código pertenece al prototipo de validación.
# En una silla de ruedas real, el L298N se sustituiría por una etapa
# de potencia de 24 V con protecciones y frenado seguro.

# ================= CONFIGURACIÓN BLE =================

# UUIDs del servicio y característica BLE publicados por el Arduino Nano.
# El ESP32 actúa como cliente BLE: busca el guante, se conecta y se suscribe
# a la característica de control que envía la trama [Mov, Vel, Giro, Status].
SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9F")
CHAR_UUID    = bluetooth.UUID("6E40000A-B5A3-F393-E0A9-E50E24DCCA9E")

# ================= CONFIGURACIÓN PINES =================

# Pines del ESP32-S3 conectados a las entradas del driver L298N.
# Cada motor utiliza dos señales: una para avance y otra para retroceso.
# En este prototipo solo se usa avance; las entradas de retroceso se mantienen a 0.
PIN_MOTOR_L_FWD = 13
PIN_MOTOR_L_BCK = 14
PIN_MOTOR_R_FWD = 11
PIN_MOTOR_R_BCK = 12

# ================= CONFIGURACIÓN DE CONTROL =================

# Zona muerta del giro alrededor del punto central.
# rxGiro llega normalizado entre 0 y 100:
#   50  -> avance recto
#   0   -> máximo giro a la izquierda
#   100 -> máximo giro a la derecha
# La zona muerta evita que pequeñas oscilaciones de la muñeca provoquen zigzagueo.
DEADZONE_GIRO = 7

# Curva de dirección aplicada al giro.
#   1.0  -> respuesta lineal
#   >1.0 -> menos sensible cerca del centro y más fuerte al final
#   <1.0 -> más sensible cerca del centro
# Se usa para que el robot sea más estable en línea recta sin perder capacidad de giro.
STEERING_CURVE = 1.2

# Tiempo máximo permitido sin recibir tramas BLE.
# Si se supera, se paran los motores. Es una protección tipo "hombre muerto"
# frente a pérdida de comunicación o bloqueo del guante.
TIMEOUT_MS = 150

# ================= MOTORES =================

# Configuración PWM de las cuatro señales del L298N.
# La frecuencia de 1000 Hz reduce el ruido audible respecto a frecuencias PWM bajas
# y da una respuesta suficiente para los motores DC pequeños del prototipo.
motor_l_fwd = PWM(Pin(PIN_MOTOR_L_FWD), freq=1000)
motor_l_bck = PWM(Pin(PIN_MOTOR_L_BCK), freq=1000)
motor_r_fwd = PWM(Pin(PIN_MOTOR_R_FWD), freq=1000)
motor_r_bck = PWM(Pin(PIN_MOTOR_R_BCK), freq=1000)

def clamp(value, min_value, max_value):
    """Limita un valor dentro de un rango seguro."""
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value

def set_motors(speed_left, speed_right):
    """
    Aplica la velocidad calculada a los dos motores del prototipo.

    Las velocidades se reciben en escala 0-255, pero MicroPython usa duty_u16
    en escala 0-65535. Por eso se realiza una conversión proporcional.
    En este prototipo no se activa el retroceso para evitar giros bruscos tipo tanque.
    """

    speed_left = clamp(int(speed_left), 0, 255)
    speed_right = clamp(int(speed_right), 0, 255)

    # Conversión de resolución: 8 bits de consigna a 16 bits de PWM.
    duty_l = int((speed_left / 255) * 65535)
    duty_r = int((speed_right / 255) * 65535)

    # Motor izquierdo: avance activo, retroceso desactivado.
    motor_l_fwd.duty_u16(duty_l)
    motor_l_bck.duty_u16(0)

    # Motor derecho: avance activo, retroceso desactivado.
    motor_r_fwd.duty_u16(duty_r)
    motor_r_bck.duty_u16(0)

def stop_motors():
    """Parada inmediata de todas las salidas PWM del driver."""
    motor_l_fwd.duty_u16(0)
    motor_l_bck.duty_u16(0)
    motor_r_fwd.duty_u16(0)
    motor_r_bck.duty_u16(0)

def compute_steering(rxGiro):
    """
    Convierte el giro recibido del guante a una variable normalizada de dirección.

    Entrada:
      rxGiro = 0...100, donde 50 es el centro.

    Salida:
      steering = -1.0...+1.0
        -1.0 -> máximo giro izquierda
         0.0 -> avance recto
        +1.0 -> máximo giro derecha

    Además se aplica zona muerta y una curva no lineal para mejorar la estabilidad.
    """

    rxGiro = clamp(int(rxGiro), 0, 100)

    # Si el valor está cerca del centro, se fuerza a avance recto.
    # Esto evita que el robot corrija continuamente por pequeñas variaciones de la mano.
    if abs(rxGiro - 50) <= DEADZONE_GIRO:
        return 0.0, 50

    # Giro a la izquierda: se mapea desde [0, 50-DEADZONE] hasta [-1, 0].
    if rxGiro < 50:
        steering = (rxGiro - (50 - DEADZONE_GIRO)) / (50 - DEADZONE_GIRO)
        giro_filtrado = rxGiro

    # Giro a la derecha: se mapea desde [50+DEADZONE, 100] hasta [0, +1].
    else:
        steering = (rxGiro - (50 + DEADZONE_GIRO)) / (50 - DEADZONE_GIRO)
        giro_filtrado = rxGiro

    steering = clamp(steering, -1.0, 1.0)

    # Curva de dirección manteniendo el signo.
    # Con STEERING_CURVE > 1, los giros pequeños son más suaves y los giros máximos
    # siguen llegando al valor completo.
    if steering >= 0:
        steering_curve = steering ** STEERING_CURVE
    else:
        steering_curve = -((-steering) ** STEERING_CURVE)

    steering_curve = clamp(steering_curve, -1.0, 1.0)

    return steering_curve, giro_filtrado

def differential_mix(rxVel, rxGiro):
    """
    Calcula la velocidad de cada motor mediante mezcla diferencial.

    El robot no tiene dirección mecánica. Gira variando la velocidad relativa
    entre la rueda izquierda y la derecha.

    Criterio usado:
      - Girar a la izquierda: se reduce la rueda izquierda.
      - Girar a la derecha: se reduce la rueda derecha.
      - No se invierte ninguna rueda, por seguridad y suavidad de movimiento.
    """

    rxVel = clamp(int(rxVel), 0, 255)

    steering, giro_filtrado = compute_steering(rxGiro)

    left_speed = rxVel
    right_speed = rxVel

    if steering < 0:
        # Giro izquierda: la rueda exterior mantiene velocidad y la interior se reduce.
        reduction = -steering
        left_speed = int(rxVel * (1.0 - reduction))
        right_speed = rxVel

    elif steering > 0:
        # Giro derecha: se reduce la rueda derecha.
        reduction = steering
        left_speed = rxVel
        right_speed = int(rxVel * (1.0 - reduction))

    left_speed = clamp(left_speed, 0, 255)
    right_speed = clamp(right_speed, 0, 255)

    return left_speed, right_speed, steering, giro_filtrado

# ================= BLE =================

# Inicialización del módulo Bluetooth Low Energy del ESP32.
ble = bluetooth.BLE()
ble.active(True)

# Códigos de eventos BLE usados por MicroPython.
# Se emplean para reaccionar ante escaneo, conexión, desconexión,
# descubrimiento de servicios/características y recepción de notificaciones.
_IRQ_SCAN_RESULT = 5
_IRQ_SCAN_DONE = 6
_IRQ_PERIPHERAL_CONNECT = 7
_IRQ_PERIPHERAL_DISCONNECT = 8
_IRQ_GATTC_SERVICE_RESULT = 9
_IRQ_GATTC_CHARACTERISTIC_RESULT = 11
_IRQ_GATTC_NOTIFY = 18

# Variables de estado de la conexión BLE.
found_device = None
conn_handle = None
value_handle = None
is_connected = False
last_packet_ms = time.ticks_ms()

def ble_irq(event, data):
    """
    Gestor principal de eventos BLE.

    El control se organiza por interrupciones/eventos, no por sondeo continuo.
    Esto permite que el ESP32 actúe justo cuando recibe una nueva trama del guante.
    """
    global found_device, conn_handle, value_handle, is_connected, last_packet_ms

    if event == _IRQ_SCAN_RESULT:
        addr_type, addr, adv_type, rssi, adv_data = data

        # Durante el escaneo se busca el UUID del servicio del guante.
        if bytes(SERVICE_UUID) in bytes(adv_data):
            print(">> Guante encontrado")
            found_device = (addr_type, bytes(addr))
            ble.gap_scan(None)

    elif event == _IRQ_SCAN_DONE:
        # Si se ha localizado el guante, se inicia la conexión.
        # Si no, se vuelve a escanear para que el sistema pueda recuperarse solo.
        if found_device:
            print(">> Conectando al guante...")
            ble.gap_connect(found_device[0], found_device[1])
        else:
            print(">> Guante no encontrado. Reintentando escaneo...")
            ble.gap_scan(2000, 30000, 30000)

    elif event == _IRQ_PERIPHERAL_CONNECT:
        conn_handle, _, _ = data
        is_connected = True
        last_packet_ms = time.ticks_ms()

        print(">> Conectado. Buscando servicios...")
        ble.gattc_discover_services(conn_handle)

    elif event == _IRQ_PERIPHERAL_DISCONNECT:
        # Ante cualquier desconexión, el receptor pasa a estado seguro.
        print(">> Desconectado. Motores OFF.")
        is_connected = False
        conn_handle = None
        value_handle = None
        found_device = None

        stop_motors()

        # Se reinicia el escaneo para reconectar automáticamente cuando el guante vuelva.
        print(">> Reiniciando escaneo BLE...")
        ble.gap_scan(2000, 30000, 30000)

    elif event == _IRQ_GATTC_SERVICE_RESULT:
        conn_handle_evt, start_handle, end_handle, uuid = data

        # Una vez encontrado el servicio del guante, se buscan sus características.
        if uuid == SERVICE_UUID:
            print(">> Servicio encontrado. Buscando característica de control...")
            ble.gattc_discover_characteristics(conn_handle_evt, start_handle, end_handle)

    elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
        conn_handle_evt, def_handle, val_handle, properties, uuid = data

        # Se localiza la característica que contiene la trama de control.
        if uuid == CHAR_UUID:
            print(">> Característica de control encontrada. Suscribiendo...")
            value_handle = val_handle

            # Activar notificaciones GATT escribiendo 0x0100 en el descriptor CCCD.
            # En este montaje el descriptor suele estar justo después de la característica.
            ble.gattc_write(conn_handle_evt, value_handle + 1, b'\x01\x00', 1)

    elif event == _IRQ_GATTC_NOTIFY:
        conn_handle_evt, val_handle, notify_data = data

        # Cada notificación recibida contiene una nueva consigna del guante.
        if value_handle is not None and val_handle == value_handle:
            last_packet_ms = time.ticks_ms()
            procesar_datos(notify_data)

# ================= LÓGICA DE CONTROL =================

def procesar_datos(data):
    """
    Procesa la trama binaria enviada por el Arduino Nano.

    Formato esperado:
      Byte 0: Movimiento
        0 = parada
        1 = movimiento activo

      Byte 1: Velocidad
        0-255

      Byte 2: Giro
        0-100, con 50 como centro

      Byte 3: Status
        0 = sistema no calibrado o no activo
        1 = sistema activo

    La lógica de seguridad se evalúa antes de aplicar cualquier PWM a los motores.
    """

    # Si la trama no tiene la longitud mínima, se descarta y se paran motores.
    if len(data) < 4:
        stop_motors()
        print("TRAMA INVÁLIDA: longitud insuficiente")
        return

    rxMov = data[0]
    rxVel = data[1]
    rxGiro = data[2]
    rxStatus = data[3]

    # Seguridad 1: si el guante no ha finalizado calibración o no está activo,
    # el receptor ignora cualquier velocidad recibida.
    if rxStatus == 0:
        stop_motors()
        print("ESPERANDO CALIBRACIÓN... Mov:{} Vel:{} Giro:{}".format(rxMov, rxVel, rxGiro))
        return

    # Seguridad 2: orden explícita de parada desde el guante.
    if rxMov == 0:
        stop_motors()
        print("PARADO")
        return

    # Si el sistema está activo, se calcula la mezcla diferencial y se aplica PWM.
    left_speed, right_speed, steering, giro_filtrado = differential_mix(rxVel, rxGiro)

    set_motors(left_speed, right_speed)

    # Mensaje de depuración para comprobar en consola la relación entre entrada BLE
    # y velocidades aplicadas a cada motor.
    print(
        "ACTIVO - Vel:{} Giro:{}->{} Steer:{:.2f} | MOT:[L:{} R:{}]".format(
            rxVel,
            rxGiro,
            giro_filtrado,
            steering,
            left_speed,
            right_speed
        )
    )

# ================= ARRANQUE =================

# El sistema arranca siempre con motores parados.
stop_motors()

# Registro del gestor de eventos BLE e inicio del escaneo.
ble.irq(ble_irq)

print("Iniciando escaneo BLE...")
ble.gap_scan(20000, 30000, 30000)

# ================= BUCLE PRINCIPAL =================

while True:
    # Watchdog por pérdida de paquetes BLE.
    # Aunque la conexión siga activa, si dejan de llegar tramas durante TIMEOUT_MS
    # se detienen los motores. Esto cubre fallos de comunicación o bloqueos del emisor.
    if is_connected:
        now = time.ticks_ms()

        if time.ticks_diff(now, last_packet_ms) > TIMEOUT_MS:
            stop_motors()
            print("TIMEOUT BLE: motores OFF")
            last_packet_ms = now

    # Periodo ligero de supervisión. El control principal se ejecuta por eventos BLE.
    time.sleep_ms(20)
