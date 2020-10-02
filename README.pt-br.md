[![forthebadge](https://forthebadge.com/images/badges/built-with-science.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/its-not-a-lie-if-you-believe-it.svg)](https://forthebadge.com)

# Simulação de VSS em ROS com Gazebo

Projeto de simulação de um time IEEE VSS em um campo oficial em ROS utilizando Gazebo

- [Simulação de VSS em ROS com Gazebo](#simulação-de-vss-em-ros-com-gazebo)
  - [📷 Screenshots](#-screenshots)
    - [Simulação de um robô](#simulação-de-um-robô)
    - [Simulação do time](#simulação-do-time)
    - [Simulação da partida](#simulação-da-partida)
  - [🎈 Introdução](#-introdução)
  - [📣 Tópicos ROS](#-tópicos-ros)
    - [⬅ Entrada](#-entrada)
    - [➡ Saída](#-saída)
  - [📏 Modelos utilizados](#-modelos-utilizados)
    - [© Crie seu próprio modelo](#-crie-seu-próprio-modelo)
  - [🔧 Parâmetros](#-parâmetros)
    - [Roslaunch](#roslaunch)
  - [📁 Estrutura de pastas](#-estrutura-de-pastas)
  - [➕ Dependências](#-dependências)
    - [🐍 Python virtual enviroment](#-python-virtual-enviroment)
  - [🎨 Cores no Gazebo](#-cores-no-gazebo)

## 📷 Screenshots

### Simulação de um robô

![screenshot](./docs/screenshot_robot.png)

### Simulação do time

![screenshot](./docs/screenshot_team.png)

### Simulação da partida

![screenshot](./docs/screenshot_match.png)

## 🎈 Introdução

É necessário clonar o projeto dentro de um workspace catkin. Para criar um workspace, veja [esse link](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

Para rodar a simulação com um robô controlável, digite:

```bash
roslaunch vss_simulation simulation_robot.launch
```

Para rodar a simulação com o time completo, digite:

```bash
roslaunch vss_simulation simulation_team.launch
```

Para rodar a simulação de uma partida, digite:

```bash
roslaunch vss_simulation simulation_match.launch
```

## 📣 Tópicos ROS

### ⬅ Entrada

A simulação suporta controle por meio de comandos de **torque** (por meio da interface **effort_controller**) ou comandos de **velocidade angular** (por meio da interface **velocity_controller**) para os dois motores de cada um dos robôs. Ambas as insterfaces estão disponíveis na biblioteca [ros_control](http://wiki.ros.org/ros_control)

Para simular robôs sem controle da rotação em malha fechada, o controle por meio do **torque** é mais adequado, uma vez que o torque é aproximadamente proporcial à tensão aplicada nos terminais de um motor DC.

Caso contrário, a interface de controle por **velocidade angular** é a mais adequada.

Em ambos os casos, os comandos são lidos nos tópicos do tipo [std_msgs/Float64](http://docs.ros.org/melodic/api/std_msgs/html/msg/Float64.html)

- **/robot[1..3]/vss_robot_left_controller/command**
- **/robot[1..3]/vss_robot_right_controller/command**
- **/foe[1..3]/vss_robot_left_controller/command**
- **/foe[1..3]/vss_robot_right_controller/command**

### ➡ Saída

Por padrão, o Gazebo publica no tópico **/gazebo/model_states** do tipo [gazebo_msgs/ModelStates](http://docs.ros.org/melodic/api/gazebo_msgs/html/msg/ModelStates.html), com uma lista de informações acerca de cada um dos modelos presentes na simulação.

```c
# broadcast all model states in world frame
string[] name                 # model names
geometry_msgs/Pose[] pose     # desired pose in world frame
geometry_msgs/Twist[] twist   # desired twist in world frame
```

Por comodidade, este pacote possui um script ([vision_proxy.py](./scripts/vision_proxy.py)) que se inscreve nesse tópico e republica a informação diferentes tópicos do tipo [gazebo_msgs/ModelState](http://docs.ros.org/melodic/api/gazebo_msgs/html/msg/ModelState.html) para cada entidade (3 robôs, 3 adversários e 1 bola, 7 tópicos no total)

```c
# Set Gazebo Model pose and twist
string model_name           # model to set state (pose and twist)
geometry_msgs/Pose pose     # desired pose in reference frame
geometry_msgs/Twist twist   # desired twist in reference frame
string reference_frame      # set pose/twist relative to the frame of this entity (Body/Model)
                            # leave empty or "world" or "map" defaults to world-frame
```

Os tópicos republicados são

- **/vision/robot[1...3]** - Tópicos para os robôs do nosso time
- **/vision/foe[1...3]** - Tópicos para os robôs adversários
- **/vision/ball** - Tópico para a bola

Todas as unidades estão no SI, distâncias estão em metros, ângulos estão em radianos, velocidade linear está em m/s e velocidade angular estã em rad/s

## 📏 Modelos utilizados

A simulação é construída a partir de um modelo de robô de VSS genérico, inspirado no modelo do [VSS SDK model](https://github.com/VSS-SDK/VSS-SDK)

Como suporte, foram criados modelos para o campo do VSS e para a bola de golf utilizada na partida, ambos construídos a partir das [regras da Robocore](https://www.robocore.net/modules.php?name=Forums&file=download&id=1424) para IEEE VSS.

### © Crie seu próprio modelo

Para criar um modelo urdf do seu projeto, você pode utilizar as ferramentas

- [Phobos](https://github.com/dfki-ric/phobos) - Gera arquivos urdf files a partir do Blender
- [SW2URDF](http://wiki.ros.org/sw_urdf_exporter) - Gera arquivos urdf a partir do SolidWorks
- [fusion2urdf](https://github.com/syuntoku14/fusion2urdf) - Gera arquivos urdf a partir do Fusion 360
- [ROS wiki](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch) - Como criar um modelo urdf do zero

Para usar seu modelo customizado, altere o valor do parâmetro ```model``` ao iniciar a simulação

## 🔧 Parâmetros

### Roslaunch

- ```model``` - Caminho do modelo do robô simulado, padrão "./urdf/vss_robot.xacro"
- ```debug``` - Habilita mensagens de debug no terminal, padrão "false"
- ```gui``` - Habilita janela GUI do Gazebo, padrão "true"
- ```paused``` - Inicia a simulação com pause, padrão "true"
- ```use_sim_time``` - Utiliza o tempo da simulação como referências das msgs, padrão "true"
- ```recording``` - Habilita o log de estados do Gazebo, padrão "false"
- ```keyboard``` - Habilita o node do controle pelo teclado/joystick, padrão "false"

Para passar um parâmetro na execução da simulação, basta escrever o nome do parâmetro separado do novo valor com ```:=```

Por exemplo, para mudar o parâmetro ```keyboard``` para ```true```:

```bash
roslaunch vss_simulation simulation_team.launch keyboard:=true
```

## 📁 Estrutura de pastas

- **docs/** - Arquivos de documentação
- **launch/** - Arquivos do [roslaunch](http://wiki.ros.org/roslaunch) escritos na [sintaxe XML](http://wiki.ros.org/roslaunch/XML) do ROS
- **meshes/** - Arquivos .stl do modelo dos nossos robôs, gerados com a extensão [SW2URDF](http://wiki.ros.org/sw_urdf_exporter) do SolidWorks
- **models/** - [Modelos personalizados para Gazebo](http://gazebosim.org/tutorials?tut=build_model) utilizados na simulação, como o campo e a bola do VSS
- **scripts/** - Rotinas python usadas no projeto
  - keyboard_node.py - Rotina para capturar a entrada do teclado ou de um joystick para controlar a simulação.
  - velocity_proxy.py - Rotina para converter a entrada recebida pelo controlador em uma mensagem de velocidade para cada motor.
  - vision_proxy.py - Rotina para separar a informação de estado do Gazebo em tópicos diferentes para cada modelo (robôs e bola).
- **urdf/** - Arquivos de descrição dos robôs no formato [.urdf](http://wiki.ros.org/urdf/XML) e [.xacro](http://wiki.ros.org/xacro). Os arquivos .urdf gerados com a extensão [SW2URDF](http://wiki.ros.org/sw_urdf_exporter) do SolidWorks
- **worlds/** - Arquivos .world no formato [SDL](http://sdformat.org/)

## ➕ Dependências

A simulação é desenvolvida para ROS e Gazebo, é recomendável instalar ambos com o comando:

```bash
sudo apt install ros-melodic-desktop-full
```

O projeto depende do pacote velocity_controllers e do effort_controllers dentro da biblioteca [ros_controllers](https://github.com/ros-controls/ros_controllers) e da biblioteca python [pygame](https://github.com/pygame/pygame). É possível instalar com ```apt-get```

```bash
sudo apt install ros-melodic-velocity-controllers ros-melodic-effort-controllers python-pygame
```

Ou usando ```rosdep```

```bash
rosdep install vss_simulation
```

### 🐍 Python virtual enviroment

Você pode querer rodar o projeto dentro de um ambiente virtual de python ([python virtualenv](https://docs.python.org/3/tutorial/venv.html)), afinal, essa é uma boa prática listada no livro de bolso de desenvolvimento python

Você pode criar um novo ambiente virtual com o comando

```sh
python3 -m venv venv
```

Em seguida, você deve rodar ```source``` para ativar o ambiente virtual

```sh
source ./venv/bin/activate
```

Para instalar as dependências, rode o comando

```sh
pip install -r requirements.txt
```

Algumas biblitecas externas podem estar faltando para [compilar](https://stackoverflow.com/questions/7652385/where-can-i-find-and-install-the-dependencies-for-pygame) o pacote ```pygame```. Você pode instalar tudo com o comando

```sh
sudo apt-get install
  subversion \
  ffmpeg \
  libsdl1.2-dev \
  libsdl-image1.2-dev \
  libsdl-mixer1.2-dev \
  libsdl-ttf2.0-dev \
  libavcodec-dev \
  libavformat-dev \
  libportmidi-dev \
  libsmpeg-dev \
  libswscale-dev \
```

## 🎨 Cores no Gazebo

Para uma lista das cores disponíveis no Gazebo, confira o arquivo de configuração do [repo oficial](https://bitbucket.org/osrf/gazebo/src/gazebo11/media/materials/scripts/gazebo.material). Temos também 2 scripts OGRE [team blue](./media/materials/scripts/team_blue.material.material) e [team yellow](./media/materials/scripts/team_yellow.material.material) para a definição de cores customizadas ([ref Gazebo](http://gazebosim.org/tutorials?tut=color_model) e [ref OGRE](http://wiki.ogre3d.org/Materials)).
