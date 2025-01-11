import argparse
from pymavlink import mavutil

def main(baudrate, device):

    the_connection = mavutil.mavlink_connection(device, baud=baudrate)

    the_connection.wait_heartbeat()
    print(f"Heartbeat recebido do sistema (system {the_connection.target_system} component {the_connection.target_component})")

    TARGET_ALTITUDE = 0.1  # Altitude desejada em metros
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,   #parece ter um parâmetro a mais
        0,  # Confirmation (0 para padrão)
        0, 0, 0, 0,  # Parâmetros ignorados
        0, 0,  # Latitude e Longitude, opcional para posição inicial
        TARGET_ALTITUDE # Puxa a altitude
    )

    print(f"Comando de decolagem enviado. Subindo para {TARGET_ALTITUDE} metros.")

    # Enviando a posição desejada no frame MAV_FRAME_LOCAL_NED
    TARGET_X = 0  # Coordenada X em metros
    TARGET_Y = 0.0   # Coordenada Y em metros
    TARGET_Z =  0 # Coordenada Z em metros (negativo para altitude acima do solo)

    the_connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10,  # Tempo de vida da mensagem
            the_connection.target_system,
            the_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b010111111000),  # Tipo de máscara (ignorar velocidades e acelerações)
            TARGET_X, TARGET_Y, TARGET_Z,  # Posição destino
            0, 0, 0,  # Velocidade (opcional)
            0, 0, 0,  # Aceleração (opcional)
            1.57,  # Yaw (direção)
            0.5    # Yaw rate (opcional)
        )
    )

    print(f"Movendo para posição relativa (X: {TARGET_X}, Y: {TARGET_Y}, Z: {TARGET_Z}).")

    # Loop para monitorar as mensagens de posição local
    while True:
        msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg:
            print(f"Posição atual: X={msg.x:.2f}, Y={msg.y:.2f}, Z={msg.z:.2f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Script para controlar drone via MAVLink")
    parser.add_argument("--baudrate", type=int, required=True, help="Baudrate para a conexão")
    parser.add_argument("--device", type=str, required=True, help="Dispositivo para conexão (ex: /dev/ttyACM0)")
    args = parser.parse_args()
    
    main(args.baudrate, args.device)
