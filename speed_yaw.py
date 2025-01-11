import argparse
from pymavlink import mavutil

parser = argparse.ArgumentParser(description="Script para controlar o MAVLink.")
parser.add_argument('--baudrate', type=int, required=True, help='Taxa de baudrate para a conexão.')
parser.add_argument('--device', type=str, required=True, help='Dispositivo para a conexão (ex: /dev/ttyACM0).')
args = parser.parse_args()

the_connection = mavutil.mavlink_connection(device=args.device, baud=args.baudrate)

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % 
      (the_connection.target_system, the_connection.target_component))

# Enviar comandos para o sistema
# 45°, taxa de  25°
# param3 = -1 permite o sentido anti-horario
# param4  = 1 permite reutilizar o programa
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 45, 25, -1, 1, 0, 0, 0)

# param2 = 5m/s 
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0, 5, 0, 0, 0, 0, 0)
