#!/usr/bin/python3

import sys
import argparse
from pymavlink import mavutil

def get_autopilot_info(mav_connection):
    msg = mav_connection.recv_match(type='HEARTBEAT', blocking=True)
    if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        return {"autopilot": "ardupilotmega"}
    elif msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
        return {"autopilot": "px4"}
    else:
        return {"autopilot": "unknown"}

def arm(mav_connection, arm_command):
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
        msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(f"Resposta recebida: {msg}")
        return msg.result
    except Exception as e:
        print(f"Erro ao enviar o comando: {e}")
        return None


# Voar definindo altitude no terminal
def takeoff(mav_connection, takeoff_altitude: float, tgt_sys_id: int = 1, tgt_comp_id: int = 1):
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    autopilot_info = get_autopilot_info(mav_connection)

    if autopilot_info["autopilot"] == "ardupilotmega":
        print("Conectado ao piloto automático ArduPilot")
        mode_id = mav_connection.mode_mapping()["GUIDED"]
        takeoff_params = [0, 0, 0, 0, 0, 0, takeoff_altitude]
    elif autopilot_info["autopilot"] == "px4":
        print("Conectado ao piloto automático PX4")
        mode_id = mav_connection.mode_mapping()["TAKEOFF"]
        msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        starting_alt = msg.alt / 1000
        takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + takeoff_altitude]
    else:
        raise ValueError("Piloto automático não suportado")

    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                         0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                         mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"ACK de Alteração de Modo: {ack_msg}")

    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                         0, 1, 0, 0, 0, 0, 0, 0)
    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"ACK de Armamento: {arm_msg}")

    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                         0, takeoff_params[0], takeoff_params[1], takeoff_params[2],
                                         takeoff_params[3], takeoff_params[4], takeoff_params[5],
                                         takeoff_params[6])
    takeoff_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"ACK de Decolagem: {takeoff_msg}")

    return takeoff_msg.result

def land(mav_connection, tgt_sys_id: int = 1, tgt_comp_id: int = 1):

    #Comando para aterrizagem
    try:
        print("Enviando comando de aterrissagem...")
        mav_connection.mav.command_long_send(
            tgt_sys_id,
            tgt_comp_id,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        land_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"ACK de Aterrissagem: {land_msg}")
        return land_msg.result
    except Exception as e:
        print(f"Erro ao enviar o comando de aterrissagem: {e}")
        return None

def main():
    parser = argparse.ArgumentParser(description="Send arm/disarm commands using MAVLink protocol.")
    parser.add_argument('--baudrate', type=int, required=True, help="Baudrate for connection")
    parser.add_argument('--device', type=str, required=True, help="Device path for connection (e.g., /dev/ttyACM0)")
    parser.add_argument('-a', '--arm', type=int, choices=[0, 1], default=1, help="Arm (1) or Disarm (0) the vehicle")
    parser.add_argument('--takeoff-altitude', type=float, help="Takeoff altitude in meters")
    parser.add_argument('--land', action='store_true', help="Land the vehicle")

    args = parser.parse_args()

    try:
        print(f"Conectando ao dispositivo {args.device} com baudrate {args.baudrate}...")
        mav_connection = mavutil.mavlink_connection(device=args.device, baud=args.baudrate)
        mav_connection.wait_heartbeat()
        print(f"Conexão estabelecida com sucesso: Sistema {mav_connection.target_system}, Componente {mav_connection.target_component}")

        if args.arm:
            result = arm(mav_connection, args.arm)
            if result == 0:
                print("Comando de armar executado com sucesso!")
            else:
                print(f"Erro ao executar o comando de armar: Código de retorno {result}")

        if args.takeoff_altitude and args.takeoff_altitude > 0:
            takeoff_result = takeoff(mav_connection, args.takeoff_altitude)
            if takeoff_result == 0:
                print("Decolagem realizada com sucesso!")
            else:
                print(f"Erro ao realizar a decolagem: Código de retorno {takeoff_result}")

        if args.land:
            land_result = land(mav_connection)
            if land_result == 0:
                print("Aterrissagem realizada com sucesso!")
            else:
                print(f"Erro ao realizar a aterrissagem: Código de retorno {land_result}")

    except Exception as e:
        print(f"Erro: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
