# Changelog

Para a versão em PT-BR 🇧🇷 desse documento, veja [aqui](./CHANGELOG.pt-br.md)

## v2.0.0

- Upgrade to Gazebo 11 and ROS noetic
- Simulation environmet released as free software
- Add MIT license to the project
- Add generic vss robot model to the simulation
- Remove original model of ThunderVolt team
- Add Gazebo camera plugin
- Add requirements.txt for python virtual environments

### Roadmap

- Creation of referee plugin for the environment

### Known problems

- Ball physical properties
- The simulation fails to start sometimes

## v2.0.0-alpha

- Compatibility break with v1.0.0
- Topics receive the command of **torque in Nm**
  - /robotX/vss_robot_left_controller/command
  - /robotX/vss_robot_right_controller/command
- Name standardization with VSSVision
- Simulation of dry and viscous friction in wheels movement
- Add ```effort_controllers``` as dependency

### Known problems

- Ball physical properties
- The simulation fails to start sometimes

## v1.0.0 - Iron Cup 2020

- First working version
- Mechanical project v1.1 - Thalles e Diego
- Motors control with **velocity in rad/s**
  - /robot_X/vss_robot_left_controller/command
  - /robot_X/vss_robot_right_controller/command
- Simulation environments
  - Single robot
  - Team of 3
  - Match 3x3
- Add ```velocity_controllers``` as dependency

### Known problems

- Propriedades físicas da bola
- Comportamento dos motores ideal demais - longe da realidade
