#!/usr/bin/python3

import sys
import argparse
from pymavlink import mavutil

def arm(mav_connection, arm_command):
    """
    Envia o comando de arm/disarm via MAVLink.
    """
    try:
        print(f"Enviando comando {'Armar' if arm_command == 1 else 'Desarmar'}...")
        mav_connection.mav.command_long_send(
            mav_connection.target_system, 
            mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
            0, 
            arm_command, 
            0, 
            0, 
            0, 
            0, 
            0, 
            0
        )

        # Aguarda uma resposta do drone
        msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(f"Resposta recebida: {msg}")
        return msg.result

    except Exception as e:
        print(f"Erro ao enviar o comando: {e}")
        return None

def main():
    """
    Ponto de entrada principal do script.
    """
    parser = argparse.ArgumentParser(description="Send arm/disarm commands using MAVLink protocol.")
    parser.add_argument('--baudrate', type=int, required=True, help="Baudrate for connection")
    parser.add_argument('--device', type=str, required=True, help="Device path for connection (e.g., /dev/ttyACM0)")
    parser.add_argument('-a', '--arm', type=int, choices=[0, 1], default=1, help="Arm (1) or Disarm (0) the vehicle")

    args = parser.parse_args()

    try:
        print(f"Conectando ao dispositivo {args.device} com baudrate {args.baudrate}...")
        mav_connection = mavutil.mavlink_connection(device=args.device, baud=args.baudrate)
        mav_connection.wait_heartbeat()
        print(f"Conexão estabelecida com sucesso: Sistema {mav_connection.target_system}, Componente {mav_connection.target_component}")

        # Envia comando arm/disarm
        result = arm(mav_connection, args.arm)
        if result == 0:
            print("Comando executado com sucesso!")
        else:
            print(f"Erro ao executar o comando: Código de retorno {result}")

    except Exception as e:
        print(f"Erro: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
