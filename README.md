[![forthebadge](https://forthebadge.com/images/badges/built-with-science.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/its-not-a-lie-if-you-believe-it.svg)](https://forthebadge.com)

# Simulação de VSS em ROS com Gazebo

Projeto de simulação de um time IEEE VSS em um campo oficial em ROS utilizando Gazebo

- [Simulação de VSS em ROS com Gazebo](#simulação-de-vss-em-ros-com-gazebo)
  - [Introdução](#introdução)
  - [Tópicos ROS](#tópicos-ros)
  - [Parâmetros](#parâmetros)
    - [Roslaunch](#roslaunch)
  - [Estrutura de pastas](#estrutura-de-pastas)
  - [Dependências](#dependências)
  - [Modelos utilizados](#modelos-utilizados)
  - [Cores no Gazebo](#cores-no-gazebo)
  - [Screenshots](#screenshots)
    - [Simulação de um robô](#simulação-de-um-robô)
    - [Simulação do time](#simulação-do-time)
    - [Simulação da partida](#simulação-da-partida)
  - [TODO](#todo)

## Introdução

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

## Parâmetros

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

## Estrutura de pastas

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

## Dependências

A simulação é desenvolvida para ROS e Gazebo, é recomendável instalar ambos com o comando:

```bash
sudo apt install ros-melodic-desktop-full
```

O projeto depende do pacote velocity_controllers e do effort-controllers dentro da biblioteca [ros_controllers](https://github.com/ros-controls/ros_controllers) e da biblioteca python [pygame](https://github.com/pygame/pygame). É possível instalar com ```apt-get```

```bash
sudo apt install ros-melodic-velocity-controllers ros-melodic-effort-controllers python-pygame
```

Ou usando ```rosdep```

```bash
rosdep install vss_simulation
```

## Modelos utilizados

A simulação é construída em de um modelo de robô de VSS genérico, inspirado no modelo do VSS SDK.

== Inserir imagens descrição ==

Como suporte, foram criados modelos para o campo do VSS e para a bola de golf utilizada na partida, ambos construídos a partir das [regras da Robocore](https://www.robocore.net/modules.php?name=Forums&file=download&id=1424) para IEEE VSS.

## Cores no Gazebo

Para uma lista das cores disponíveis no Gazebo, confira o arquivo de configuração do [repo oficial](https://bitbucket.org/osrf/gazebo/src/gazebo11/media/materials/scripts/gazebo.material). Temos também um [script OGRE](./media/materials/scripts/vss.material) para a definição de cores customizadas ([ref](http://gazebosim.org/tutorials?tut=color_model) do Gazebo a respeito).

## Screenshots

### Simulação de um robô

![screenshot](./docs/screenshot_robot.png)

### Simulação do time

![screenshot](./docs/screenshot_team.png)

### Simulação da partida

![screenshot](./docs/screenshot_match.png)

## TODO

- Completar documentação.
- Atualizar screenshots
