[![forthebadge](https://forthebadge.com/images/badges/built-with-science.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/its-not-a-lie-if-you-believe-it.svg)](https://forthebadge.com)

# Simulação de VSS em ROS com Gazebo

Projeto de simulação de um time IEEE VSS em um campo oficial em ROS utilizando Gazebo

- [Simulação de VSS em ROS com Gazebo](#simulação-de-vss-em-ros-com-gazebo)
  - [Introdução](#introdução)
  - [Tópicos ROS](#tópicos-ros)
  - [Câmera virtual](#câmera-virtual)
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

## Tópicos ROS

O pacore [ThunderVolt](https://github.com/ThundeRatz/vss_thundervolt) se inscreve em 7 tópicos do tipo [gazebo_msgs/ModelState](http://docs.ros.org/jade/api/gazebo_msgs/html/msg/ModelState.html) (3 robôs, 3 adversários e 1 bola)

```c
# Set Gazebo Model pose and twist
string model_name           # model to set state (pose and twist)
geometry_msgs/Pose pose     # desired pose in reference frame
geometry_msgs/Twist twist   # desired twist in reference frame
string reference_frame      # set pose/twist relative to the frame of this entity (Body/Model)
                            # leave empty or "world" or "map" defaults to world-frame
```

Por padrão o Gazebo publica 1 tópico do tipo [gazebo_msgs/ModelStates](http://docs.ros.org/jade/api/gazebo_msgs/html/msg/ModelStates.html)

```c
# broadcast all model states in world frame
string[] name                 # model names
geometry_msgs/Pose[] pose     # desired pose in world frame
geometry_msgs/Twist[] twist   # desired twist in world frame
```

Este pacote de simulação possui um script python que se increve no tópico do Gazebo e republica a informação nos 7 tópicos esperados pelo [ThunderVolt](https://github.com/ThundeRatz/vss_thundervolt):

- **/vision/robot[1...3]** - Tópicos para os robôs do nosso time
- **/vision/foe[1...3]** - Tópicos para os robôs adversários
- **/vision/ball** - Tópico para a bola

Todas as unidades estão no SI, distâncias estão em metros, ângulos estão em radianos, velocidade linear está em m/s e velocidade angular estã em rad/s

## Câmera virtual

A simulação possui uma câmera virtual que captura imagens do topo do campo, de forma semelhante ao que acontece em uma partida de VSS real.

A câmera publica as imagens obtidas no tópico **/camera/image_raw**

É possível acompanhar as imagens com o auxílio do pacote [image_view](http://wiki.ros.org/image_view)

```sh
rosrun image_view image_view image:=/camera/image_raw
```

## Parâmetros

### Roslaunch

- ```model``` - Caminho do modelo do robô simulado
  - **Padrão** "./urdf/vss_robot.xacro"
- ```debug``` - Habilita mensagens de debug no terminal
  - **Padrão** "false"
- ```gui``` - Habilita janela GUI do Gazebo
  - **Padrão** "true"
- ```paused``` - Inicia a simulação com pause
  - **Padrão** "true"
- ```use_sim_time``` - Utiliza o tempo da simulação como referências das mensagens publicadas
  - **Padrão** "true"
- ```world_name``` - Seleciona o arquivo .world a ser utilizado
  - **Padrão** "vss_field.world" - Conjunto de bola e campo em vista isométrica
  - "vss_field_top.world" - Conjunto de bola e campo em vista superior ortogonal
  - "vss_field_spots.world" - Conjunto de bola e campo em vista isométrica com fontes de luz pontuais
- ```recording``` - Habilita o log de estados do Gazebo
  - **Padrão** "false"
- ```keyboard``` - Habilita o node do controle pelo teclado/joystick
  - **Padrão** "false"

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

A simulação é construída em volta da versão 1.1 do robô de VSS do time ThunderVolt. Como suporte, foram criados modelos para o campo do VSS e para a bola de golf utilizada na partida, ambos construídos a partir das [regras da Robocore](https://www.robocore.net/modules.php?name=Forums&file=download&id=1424) para IEEE VSS.

## Cores no Gazebo

Para uma lista das cores disponíveis no Gazebo, confira o arquivo de configuração do [repo oficial](https://github.com/osrf/gazebo/blob/gazebo11/media/materials/scripts/gazebo.material). Temos também um [script OGRE](./media/materials/scripts/vss.material) para a definição de cores customizadas ([ref](http://gazebosim.org/tutorials?tut=color_model) do Gazebo a respeito).

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
