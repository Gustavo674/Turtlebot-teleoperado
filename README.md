# Turtlebot-teleoperado

## Descrição
Este projeto permite a teleoperação de um Turtlebot utilizando ROS 2 e Webots. A interface do usuário permite o controle do robô por meio de teclas, mostrando a velocidade em tempo real.

## Estrutura de Diretórios
A estrutura de diretórios do projeto é a seguinte:

Turtlebot-teleoperado/
├── src/
│ └── turtlebot_teleop/
│ ├── resource/
│ │ └── init.py
│ ├── turtlebot_teleop/
│ │ ├── init.py
│ │ ├── teleop_interface.py
│ │ └── teleop_node.py
│ ├── setup.py
│ └── setup.cfg
├── README.md
└── install/ (criado após build)

## Requisitos
- ROS 2 Humble
- Webots
- Pacotes ROS 2 para Webots e Turtlebot3

## Configuração do Projeto

1. Clone o repositório:
```bash
git https://github.com/Gustavo674/Turtlebot-teleoperado.git
cd Turtlebot-teleoperado
```

### Descrição das Pastas e Arquivos

- `src/`: Diretório que contém todos os pacotes ROS 2.
- `src/turtlebot_teleop/`: Pacote ROS 2 principal para teleoperação.
  - `resource/`: Diretório para arquivos de recursos, atualmente contém apenas o arquivo `__init__.py`.
  - `turtlebot_teleop/`: Diretório com os scripts de teleoperação.
    - `teleop_interface.py`: Script que implementa a interface de teleoperação.
    - `teleop_node.py`: Script que implementa o nó de controle do robô.
  - `setup.py`: Script de configuração para o pacote ROS 2.
  - `setup.cfg`: Arquivo de configuração complementar para o `setup.py`.
- `README.md`: Este arquivo, contendo informações sobre o projeto.

## Instalação

### Configuração do Ambiente

1. **Instale o Webots:**
   ```bash
   sudo mkdir -p /etc/apt/keyrings
   cd /etc/apt/keyrings
   sudo wget -q https://cyberbotics.com/Cyberbotics.asc
   echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
   sudo apt update
   sudo apt install webots

2. **Instale os pacotes ROS 2 necessários:**
    ```bash
    sudo apt install ros-humble-webots-ros2 ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* ros-humble-rmw-cyclonedds-cpp ros-humble-nav2-simple-commander ros-humble-tf-transformations python3-transforms3d

3. **Configure o DDS**
   ```bash
   echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
    source ~/.bashrc

4. **Atualize o arquivo burguer.yaml**

  Edite o arquivo /opt/ros/humble/share/turtlebot3_navigation2/param/burger.yaml:

  `sudo nano /opt/ros/humble/share/turtlebot3_navigation2/param/burger.yaml`
  
  Altere a linha:

  `robot_model_type: "differential"`
  
  Para:

  `robot_model_type: "nav2_amcl::DifferentialMotionModel"`

## Compilação do Workspace

1.**Navegue para o diretório raiz do workspace e compile:**
  ```bash
  cd ~/Turtlebot-teleoperado 
  colcon build
  ```
2.**Fonte o ambiente:**
  ```bash
  `source /opt/ros/humble/setup.bash
   source install/setup.bash`
  ```
## Execução

1.**Terminal 1: Iniciar Webots**
    ```bash
    cd ~/Turtlebot-teleoperado
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch webots_ros2_turtlebot robot_launch.py
    ```
2.**Terminal 2: Navegação e Mapeamento Simultâneo**
    ```bash
    cd ~/Turtlebot-teleoperado
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
    ```
3.**Terminal 3: Interface de Teleoperação**
    ```bash
    cd ~/Turtlebot-teleoperado
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run turtlebot_teleop teleop_interface
    ```
4.**Terminal 4: Nó de Controle**
     ```bash
    cd ~/Turtlebot-teleoperado
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run turtlebot_teleop teleop_node
    ```
## Controlando o Robô

Para controlar o robô, você deve estar no terminal onde a interface de teleoperação está sendo executada (ros2 run turtlebot_teleop teleop_interface). Use as seguintes teclas para controlar o robô:

w: Move para frente.
s: Move para trás.
a: Rotaciona para a esquerda.
d: Rotaciona para a direita.
q: Para o robô e sai da teleoperação.

Certifique-se de que o terminal da interface de teleoperação está em foco para que ele possa receber os comandos do teclado.

## Vídeo de Demonstração

[Link para o vídeo]()
