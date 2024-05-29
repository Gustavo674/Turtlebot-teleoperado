# Turtlebot Teleop

## Descrição
Este projeto permite a teleoperação de um Turtlebot utilizando ROS 2 e Webots. A interface do usuário permite o controle do robô por meio de teclas, mostrando a velocidade em tempo real.

## Estrutura de Diretórios
A estrutura de diretórios do projeto é a seguinte:

| Diretório/Arquivo                       | Descrição                                                |
|-----------------------------------------|----------------------------------------------------------|
| `Turtlebot-teleoperado/`                | Diretório raiz do projeto                                |
| `├── src/`                              | Contém todos os pacotes ROS 2                            |
| `│   └── turtlebot_teleop/`             | Pacote ROS 2 principal para teleoperação                 |
| `│       ├── resource/`                 | Diretório para arquivos de recursos                      |
| `│       │   └── __init__.py`           | Arquivo de inicialização do pacote                       |
| `│       ├── turtlebot_teleop/`         | Diretório com os scripts de teleoperação                 |
| `│       │   ├── __init__.py`           | Arquivo de inicialização do pacote                       |
| `│       │   ├── teleop_interface.py`   | Script que implementa a interface de teleoperação        |
| `│       │   ├── teleop_node.py`        | Script que implementa o nó de controle do robô           |
| `│       │   ├── image_publisher.py`    | Script que publica imagens da câmera                     |
| `│       │   ├── image_subscriber.py`   | Script que recebe e processa imagens                     |
| `│       │   ├── teleop_interface_gui.py`| Script que implementa a interface gráfica de teleoperação|
| `│       ├── setup.py`                  | Script de configuração para o pacote ROS 2               |
| `│       └── setup.cfg`                 | Arquivo de configuração complementar para o `setup.py`   |
| `├── README.md`                         | Arquivo README com informações sobre o projeto           |
| `└── install/`                          | Diretório criado após a build                            |

## Requisitos
- ROS 2 Humble
- Webots
- Pacotes ROS 2 para Webots e Turtlebot3

## Configuração do Projeto

1. Clone o repositório:
    ```bash
    git clone https://github.com/Gustavo674/Turtlebot-teleoperado.git
    cd Turtlebot-teleoperado
    ```

## Configuração do Ambiente

1. **Instale o Webots:**
    ```bash
    sudo mkdir -p /etc/apt/keyrings
    cd /etc/apt/keyrings
    sudo wget -q https://cyberbotics.com/Cyberbotics.asc
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
    sudo apt update
    sudo apt install webots
    ```

2. **Instale os pacotes ROS 2 necessários:**
    ```bash
    sudo apt install ros-humble-webots-ros2 ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3* ros-humble-rmw-cyclonedds-cpp ros-humble-nav2-simple-commander ros-humble-tf-transformations python3-transforms3d
    ```

3. **Configure o DDS:**
    ```bash
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
    source ~/.bashrc
    ```

4. **Atualize o arquivo `burger.yaml`:**
    Edite o arquivo `/opt/ros/humble/share/turtlebot3_navigation2/param/burger.yaml`:
    ```bash
    sudo nano /opt/ros/humble/share/turtlebot3_navigation2/param/burger.yaml
    ```
    Altere a linha:
    ```yaml
    robot_model_type: "differential"
    ```
    Para:
    ```yaml
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    ```

## Compilação do Workspace

1. **Navegue para o diretório raiz do workspace e compile:**
    ```bash
    cd ~/Turtlebot-teleoperado
    colcon build
    ```

2. **Fonte o ambiente:**
    ```bash
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ```

## Execução

1. **Terminal 1: Iniciar Webots**
    ```bash
    cd ~/Turtlebot-teleoperado
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch webots_ros2_turtlebot robot_launch.py
    ```

2. **Terminal 2: Navegação e Mapeamento Simultâneo**
    ```bash
    cd ~/Turtlebot-teleoperado
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
    ```

3. **Terminal 3: Interface Gráfica de Teleoperação**
    ```bash
    cd ~/Turtlebot-teleoperado
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run turtlebot_teleop teleop_interface_gui
    ```

4. **Terminal 4: Nó de Controle**
    ```bash
    cd ~/Turtlebot-teleoperado
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run turtlebot_teleop teleop_node
    ```

5. **Terminal 5: Detecção de Obstáculos**
    ```bash
    cd ~/Turtlebot-teleoperado
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run turtlebot_teleop obstacle_detection
    ```

6. **Terminal 6: Publicação de Imagens**
    ```bash
    cd ~/Turtlebot-teleoperado
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run turtlebot_teleop image_publisher
    ```

7. **Terminal 7: Assinatura de Imagens**
    ```bash
    cd ~/Turtlebot-teleoperado
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 run turtlebot_teleop image_subscriber
    ```

## Controlando o Robô

Para controlar o robô, você deve estar no terminal onde a interface gráfica de teleoperação está sendo executada (`ros2 run turtlebot_teleop teleop_interface_gui`). Use os botões na interface gráfica para controlar o robô:

- `Forward`: Move para frente.
- `Backward`: Move para trás.
- `Left`: Rotaciona para a esquerda.
- `Right`: Rotaciona para a direita.
- `Stop`: Para o robô.

A latência estimada para cada frame será exibida na interface gráfica.

## Vídeo de Demonstração
[Link para o vídeo](https://drive.google.com/file/d/1e1YzeNTJLjSPFr0Ow0qE5D45DKb8fLdS/view?usp=sharing)
