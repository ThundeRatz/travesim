[![forthebadge](https://forthebadge.com/images/badges/built-with-science.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/its-not-a-lie-if-you-believe-it.svg)](https://forthebadge.com)

# Simulação de VSS em ROS com Gazebo

Projeto de simulação de um time IEEE VSS em um campo oficial em ROS utilizando Gazebo

- [Simulação de VSS em ROS com Gazebo](#simula%c3%a7%c3%a3o-de-vss-em-ros-com-gazebo)
  - [Introdução](#introdu%c3%a7%c3%a3o)
  - [Parâmetros](#par%c3%a2metros)
    - [Simulação de um robô](#simula%c3%a7%c3%a3o-de-um-rob%c3%b4)
  - [Estrutura de pastas](#estrutura-de-pastas)
  - [Dependências](#depend%c3%aancias)
  - [Modelos utilizados](#modelos-utilizados)
  - [TODO](#todo)
  - [Screenshots](#screenshots)
    - [Simulação de um robô](#simula%c3%a7%c3%a3o-de-um-rob%c3%b4-1)
    - [Simulação do time](#simula%c3%a7%c3%a3o-do-time)

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

## Parâmetros

### Simulação de um robô

 - **model** - Caminho do modelo do robô simulado, padrão "./urdf/vss_robot.xacro"
 - **debug** - Habilita mensagens de debug no terminal, padrão "false"
 - **gui** - Habilita janela GUI do Gazebo, padrão "true"
 - **paused** - Inicia a simulação com pause, padrão "true"
 - **use_sim_time** - Utiliza o tempo da simulação como referências das msgs, padrão "true"
 - **recording** - Habilita o log de estados do Gazebo, padrão "false"
 - **keyboard** - Habilita o node do controle pelo teclado/joystick, padrão "true" para simualtion_robot e padrão "false" para simulation_team

## Estrutura de pastas

TODO

- **docs/**
- **launch/**
- **meshes/**
- **models/**
- **scripts/**
- **urdf/**
- **worlds/**

## Dependências

A simulação é desenvolvida para ROS e Gazebo, é recomendável instalar ambos com o comando:

```bash
sudo apt install ros-melodic-desktop-full
```

O projeto depende do pacote velocity_controllers dentro da biblioteca [ros_controllers](https://github.com/ros-controls/ros_controllers) e da biblioteca python [pygame](https://github.com/pygame/pygame). É possível instalar com ```apt-get```

```bash
sudo apt install ros-melodic-velocity-controllers python-pygame
```

Ou usando ```rosdep```

```bash
rosdep install vss_simulation
```

## Modelos utilizados

A simulação é construída em volta da versão 1.1 do robô de VSS do time ThunderVolt. Como suporte, foram criados modelos para o campo do VSS e para a bola de golf utilizada na partida, ambos construídos a partir das [regras da Robocore](https://www.robocore.net/modules.php?name=Forums&file=download&id=1424) para IEEE VSS.

## TODO

Completar documentação e integrar com código do [ThunderVolt](https://github.com/ThundeRatz/vss_thundervolt).

## Screenshots

### Simulação de um robô

![screenshot](./docs/screenshot_robot.png)

### Simulação do time

![screenshot](./docs/screenshot_team.png)
