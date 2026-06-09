import bluetooth
import time
from machine import Pin, PWM

# ================= CONFIGURACIÓN BLE =================

# UUIDs del guante Arduino Nano 33 BLE Sense
SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9F")
CHAR_UUID    = bluetooth.UUID("6E40000A-B5A3-F393-E0A9-E50E24DCCA9E")

# ================= CONFIGURACIÓN PINES =================

# Pines del ESP32-S3 conectados al L298N
PIN_MOTOR_L_FWD = 13
PIN_MOTOR_L_BCK = 14
PIN_MOTOR_R_FWD = 11
PIN_MOTOR_R_BCK = 12

# ================= CONFIGURACIÓN DE CONTROL =================

# Zona muerta del giro.
# rxGiro llega de 0 a 100:
# 50 = recto
# 0 = máximo izquierda
# 100 = máximo derecha
DEADZONE_GIRO = 7

# Curva de dirección:
# 1.0 = respuesta lineal
# >1.0 = más suave cerca del centro, más fuerte al final
# <1.0 = más sensible cerca del centro
STEERING_CURVE = 1.2

# Seguridad por pérdida de paquetes.
# Si no llega ninguna trama en este tiempo, se paran los motores.
TIMEOUT_MS = 150

# ================= MOTORES =================

motor_l_fwd = PWM(Pin(PIN_MOTOR_L_FWD), freq=1000)
motor_l_bck = PWM(Pin(PIN_MOTOR_L_BCK), freq=1000)
motor_r_fwd = PWM(Pin(PIN_MOTOR_R_FWD), freq=1000)
motor_r_bck = PWM(Pin(PIN_MOTOR_R_BCK), freq=1000)

def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value

def set_motors(speed_left, speed_right):
    """
    speed_left y speed_right deben estar en rango 0-255.
    En este prototipo solo se usa avance. Las entradas de retroceso se mantienen a 0.
    """

    speed_left = clamp(int(speed_left), 0, 255)
    speed_right = clamp(int(speed_right), 0, 255)

    duty_l = int((speed_left / 255) * 65535)
    duty_r = int((speed_right / 255) * 65535)

    motor_l_fwd.duty_u16(duty_l)
    motor_l_bck.duty_u16(0)

    motor_r_fwd.duty_u16(duty_r)
    motor_r_bck.duty_u16(0)

def stop_motors():
    motor_l_fwd.duty_u16(0)
    motor_l_bck.duty_u16(0)
    motor_r_fwd.duty_u16(0)
    motor_r_bck.duty_u16(0)

def compute_steering(rxGiro):
    """
    Convierte rxGiro, en escala 0-100, a steering en escala -1.0 a +1.0.

    -1.0 = máximo giro izquierda
     0.0 = recto
    +1.0 = máximo giro derecha

    Aplica zona muerta y curva de dirección.
    """

    rxGiro = clamp(int(rxGiro), 0, 100)

    # Zona muerta alrededor del centro
    if abs(rxGiro - 50) <= DEADZONE_GIRO:
        return 0.0, 50

    # Izquierda
    if rxGiro < 50:
        # Mapea desde [0, 50 - DEADZONE_GIRO] hasta [-1.0, 0.0]
        steering = (rxGiro - (50 - DEADZONE_GIRO)) / (50 - DEADZONE_GIRO)
        giro_filtrado = rxGiro

    # Derecha
    else:
        # Mapea desde [50 + DEADZONE_GIRO, 100] hasta [0.0, +1.0]
        steering = (rxGiro - (50 + DEADZONE_GIRO)) / (50 - DEADZONE_GIRO)
        giro_filtrado = rxGiro

    steering = clamp(steering, -1.0, 1.0)

    # Curva de dirección manteniendo el signo
    if steering >= 0:
        steering_curve = steering ** STEERING_CURVE
    else:
        steering_curve = -((-steering) ** STEERING_CURVE)

    steering_curve = clamp(steering_curve, -1.0, 1.0)

    return steering_curve, giro_filtrado

def differential_mix(rxVel, rxGiro):
    """
    Calcula la velocidad de cada motor usando mezcla diferencial.

    En este prototipo:
    - Para girar a la izquierda se reduce el motor izquierdo.
    - Para girar a la derecha se reduce el motor derecho.
    - No se invierte ningún motor, por seguridad y para evitar giro tipo tanque.
    """

    rxVel = clamp(int(rxVel), 0, 255)

    steering, giro_filtrado = compute_steering(rxGiro)

    left_speed = rxVel
    right_speed = rxVel

    if steering < 0:
        # Giro izquierda: reducir rueda izquierda
        reduction = -steering
        left_speed = int(rxVel * (1.0 - reduction))
        right_speed = rxVel

    elif steering > 0:
        # Giro derecha: reducir rueda derecha
        reduction = steering
        left_speed = rxVel
        right_speed = int(rxVel * (1.0 - reduction))

    left_speed = clamp(left_speed, 0, 255)
    right_speed = clamp(right_speed, 0, 255)

    return left_speed, right_speed, steering, giro_filtrado

# ================= BLE =================

ble = bluetooth.BLE()
ble.active(True)

_IRQ_SCAN_RESULT = 5
_IRQ_SCAN_DONE = 6
_IRQ_PERIPHERAL_CONNECT = 7
_IRQ_PERIPHERAL_DISCONNECT = 8
_IRQ_GATTC_SERVICE_RESULT = 9
_IRQ_GATTC_CHARACTERISTIC_RESULT = 11
_IRQ_GATTC_NOTIFY = 18

found_device = None
conn_handle = None
value_handle = None
is_connected = False
last_packet_ms = time.ticks_ms()

def ble_irq(event, data):
    global found_device, conn_handle, value_handle, is_connected, last_packet_ms

    if event == _IRQ_SCAN_RESULT:
        addr_type, addr, adv_type, rssi, adv_data = data

        if bytes(SERVICE_UUID) in bytes(adv_data):
            print(">> Guante encontrado")
            found_device = (addr_type, bytes(addr))
            ble.gap_scan(None)

    elif event == _IRQ_SCAN_DONE:
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
        print(">> Desconectado. Motores OFF.")
        is_connected = False
        conn_handle = None
        value_handle = None
        found_device = None

        stop_motors()

        print(">> Reiniciando escaneo BLE...")
        ble.gap_scan(2000, 30000, 30000)

    elif event == _IRQ_GATTC_SERVICE_RESULT:
        conn_handle_evt, start_handle, end_handle, uuid = data

        if uuid == SERVICE_UUID:
            print(">> Servicio encontrado. Buscando característica de control...")
            ble.gattc_discover_characteristics(conn_handle_evt, start_handle, end_handle)

    elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
        conn_handle_evt, def_handle, val_handle, properties, uuid = data

        if uuid == CHAR_UUID:
            print(">> Característica de control encontrada. Suscribiendo...")
            value_handle = val_handle

            # Activar notificaciones escribiendo 0x0100 en el descriptor CCCD.
            # En este montaje el descriptor suele estar en val_handle + 1.
            ble.gattc_write(conn_handle_evt, value_handle + 1, b'\x01\x00', 1)

    elif event == _IRQ_GATTC_NOTIFY:
        conn_handle_evt, val_handle, notify_data = data

        if value_handle is not None and val_handle == value_handle:
            last_packet_ms = time.ticks_ms()
            procesar_datos(notify_data)

# ================= LÓGICA DE CONTROL =================

def procesar_datos(data):
    """
    Trama esperada desde el Arduino Nano:

    Byte 0: Movimiento
        0 = parada
        1 = movimiento activo

    Byte 1: Velocidad
        0-255

    Byte 2: Giro
        0-100
        50 = centro

    Byte 3: Status
        0 = sistema no calibrado / no activo
        1 = sistema activo
    """

    if len(data) < 4:
        stop_motors()
        print("TRAMA INVÁLIDA: longitud insuficiente")
        return

    rxMov = data[0]
    rxVel = data[1]
    rxGiro = data[2]
    rxStatus = data[3]

    # Seguridad 1: sistema no calibrado o no activo
    if rxStatus == 0:
        stop_motors()
        print("ESPERANDO CALIBRACIÓN... Mov:{} Vel:{} Giro:{}".format(rxMov, rxVel, rxGiro))
        return

    # Seguridad 2: orden de parada
    if rxMov == 0:
        stop_motors()
        print("PARADO")
        return

    # Movimiento activo
    left_speed, right_speed, steering, giro_filtrado = differential_mix(rxVel, rxGiro)

    set_motors(left_speed, right_speed)

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

stop_motors()

ble.irq(ble_irq)

print("Iniciando escaneo BLE...")
ble.gap_scan(20000, 30000, 30000)

# ================= BUCLE PRINCIPAL =================

while True:
    # Watchdog por pérdida de paquetes BLE.
    # Si el guante deja de enviar tramas, se paran los motores.
    if is_connected:
        now = time.ticks_ms()

        if time.ticks_diff(now, last_packet_ms) > TIMEOUT_MS:
            stop_motors()
            print("TIMEOUT BLE: motores OFF")
            last_packet_ms = now

    time.sleep_ms(20)



