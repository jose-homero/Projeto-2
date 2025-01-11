#!/usr/bin/python3

import argparse
from pymavlink import mavutil

def send_command_set_message_interval(connection, message_id, interval_us):

    #Envia o comando MAV_CMD_SET_MESSAGE_INTERVAL para configurar a frequência de uma mensagem MAVLink.

   
# ID da mensagem MAVLink que deseja configurar.
    #param interval_us: Intervalo em microssegundos (0 para desligar, -1 para usar padrão).

    command = mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
    target_system = connection.target_system
    target_component = connection.target_component

    connection.mav.command_long_send(
        target_system,                 # ID do sistema alvo
        target_component,              # ID do componente alvo
        command,                       # Comando MAVLink
        0,                             # Confirmation (0 para nenhum)
        message_id,                    # param1: ID da mensagem (ex.: MAVLINK_MSG_ID_GPS_RAW_INT)
        interval_us,                   # param2: Intervalo em microssegundos
        50000,                         # param3
        0,                             # param4
        0,                             # param5
        0,                             # param6
        0                              # param7
    )
    print(f"Comando enviado: Mensagem ID={message_id}, Intervalo={interval_us}us")


def read_gps_data(connection):

    #Lê e imprime dados de GPS da mensagem GPS_RAW_INT.

    print("Aguardando mensagem GPS_RAW_INT.")
    try:
        # Espera pela mensagem GPS_RAW_INT
        message = connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=10)
        if message is not None:
            print(f"Dados do GPS recebidos:")
            print(f"  Latitude: {message.lat / 1e7}°")
            print(f"  Longitude: {message.lon / 1e7}°")
            print(f"  Altitude (MSL): {message.alt / 1000.0} m")
            print(f"  Velocidade: {message.vel / 100.0} m/s")
            print(f"  Precisão Horizontal: {message.h_acc / 100.0} m")
            print(f"  Precisão Vertical: {message.v_acc / 100.0} m")
            print(f"  Número de Satélites: {message.satellites_visible}")
        else:
            print("Nenhuma mensagem GPS_RAW_INT recebida dentro do tempo limite.")
    except Exception as e:
        print(f"Erro ao ler dados de GPS: {e}")


def main():
    # Configuração de argumentos de linha de comando
    parser = argparse.ArgumentParser(description="Lê dados de GPS (latitude, longitude, altitude) usando MAVLink.")
    parser.add_argument('--baudrate', type=int, required=True, help="Baudrate para conexão")
    parser.add_argument('--device', type=str, required=True, help="Caminho do dispositivo (ex.: /dev/ttyACM0)")
    args = parser.parse_args()

    # Conecta ao Pixhawk
    print(f"Conectando ao dispositivo {args.device} com baudrate {args.baudrate}...")
    connection = mavutil.mavlink_connection(device=args.device, baud=args.baudrate)
    connection.wait_heartbeat()
    print("Conexão estabelecida com o Pixhawk.")

    # Envia o comando para configurar o intervalo da mensagem GPS_RAW_INT
    message_id = mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT
    interval_us = 1000000  # Intervalo de 1 segundo (em microssegundos)
    send_command_set_message_interval(connection, message_id, interval_us)

    # Lê dados de GPS
    read_gps_data(connection)


if __name__ == "__main__":
    main()
