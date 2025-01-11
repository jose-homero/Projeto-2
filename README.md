# Projeto MAVLink para Controle de Drones

Este repositório contém uma série de scripts em Python para comunicação e controle de drones usando o protocolo MAVLink. O projeto é projetado para executar tarefas como configuração de mensagens, armamento/desarmamento, controle de movimento, decolagem e orientação do drone.

---

## 📂 Estrutura do Repositório

projeto-drones/ ├── latitudeteste.py # Configuração de mensagens e leitura de GPS ├── arm.py # Armamento e desarmamento do drone ├── movement.py # Controle de movimento e decolagem ├── speed_yaw.py # Controle de orientação (Yaw) ├── README.md # Documentação do projeto

---

## 🛠️ Requisitos

- **Python 3.x**: Interpretador Python compatível.
- **Biblioteca `pymavlink`**: Comunicação via MAVLink.
  - Instale com: 
    ```bash
    pip install pymavlink
    ```
- **Dispositivo Pixhawk**: Com suporte ao protocolo MAVLink.

---

## 📝 Scripts Disponíveis

### 1. `latitudeteste.py`
Este script configura a frequência das mensagens MAVLink e realiza a leitura de dados de GPS do Pixhawk.  
**🔑 Funcionalidades principais:**
- Configuração de frequência de mensagens via `send_command_set_message_interval()`.
- Leitura de dados GPS (latitude, longitude, altitude) com `read_gps_data()`.
- Verificação de sinais vitais com `wait_heartbeat()`.

**💻 Comando para execução:**
```bash
Home/Development/latitudeteste.py --baudrate 921600 --device /dev/ttyACM0
--baudrate: Taxa de comunicação (ex.: 921600).
--device: Porta serial do drone (ex.: /dev/ttyACM0).
```

---

### 2. `arm.py`
Este script é usado para armar ou desarmar o drone, preparando-o para voo.
🔑 Funcionalidades principais:

Envio do comando MAV_CMD_COMPONENT_ARM_DISARM.
Confirmação via COMMAND_ACK.
💻 Comando para execução:
    ```bash
home/Development/arm.py --baudrate 921600 --device /dev/ttyACM0 --arm 1
    ```
--arm: Use 1 para armar o drone ou 0 para desarmá-lo.

---


### 3. `movement.py`
Este script gerencia o movimento do drone, incluindo decolagem e movimentação em coordenadas específicas.
🔑 Funcionalidades principais:

Decolagem controlada com altitude-alvo (TARGET_ALTITUDE).
Movimento em coordenadas definidas (x, y, z) no frame MAV_FRAME_LOCAL_NED.
💻 Parâmetros importantes:

x, y, z: Coordenadas de destino.
yaw: Ângulo de orientação do drone.
yaw_rate: Taxa de variação da orientação.
📖 Observações:

A máscara POSITION_TARGET_TYPEMASK é configurada para desativar velocidades e acelerações por padrão.
Ideal para movimentos precisos em missões autônomas.

---


### 4. `speed_yaw.py`
Este script controla especificamente a orientação do drone.
🔑 Funcionalidades principais:

Ajuste do ângulo de orientação (yaw).
Controle da taxa de variação do ângulo (yaw_rate).
💻 Comando utilizado no código:
    ```bash
mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, angulo, taxa_de_variacao_do_angulo, rotacao, reutilizacao_do_programa, 0, 0, 0
    ```
📖 Parâmetros importantes:

angulo: Ângulo desejado em graus.
taxa_de_variacao_do_angulo: Velocidade de rotação em graus/segundo.
rotacao: Direção da rotação (1 para horário, -1 para anti-horário).
reutilizacao_do_programa: Permite reusar o comando (1 ativado, 0 desativado).

🚀 Como Executar os Scripts
Clone o repositório:
git clone https://github.com/usuario/projeto-drones.git
cd projeto-drones
Instale as dependências: Certifique-se de que o Python 3.x e o pymavlink estão instalados:
pip install pymavlink
Conecte o drone:

Plugue o dispositivo Pixhawk na porta USB do computador.
Verifique a porta serial (ex.: /dev/ttyACM0 no Linux).
Execute o script desejado


