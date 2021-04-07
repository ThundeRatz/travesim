# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Calendar Versioning](https://calver.org/).

Para a versão em PT-BR 🇧🇷 desse documento, veja [aqui](./CHANGELOG.pt-br.md)

## [21.04.1]

### Changed

- Project name is now **TraveSim** instead of **vss_simulation**
- Project uses now calendar versioning with the format YY.0M.MINOR
- Screenshots are now in the end of the README

## [21.03.2]

### Changed

- Robots names now follow the convention /[yellow|blue]\_team/robot\_[0..2]. This change applies to robots model name and topics namespace
- The controllers topic changed as
  - **/[yellow|blue]\_team/robot\_[0..2]/~~vss_robot~~\_diff\_drive\_controller/cmd\_vel** - **/[yellow|blue]\_team/robot\_[0..2]/diff\_drive\_controller/cmd\_vel**
  - **/[yellow|blue]\_team/robot\_[0..2]/~~vss_robot~~\_[left|right]_controller/command** - **/[yellow|blue]\_team/robot\_[0..2]/[left|right]_controller/command**

## [21.03.1]

### Added

- Selectable control interface: differenctial drive controller (Twist messages) or wheels angular speed direct control

### Changed

- Migrate keyboard controller script

### Deprecated

- Wheels torque direct control

## [20.10.1] - Open Camera

### Added

- Software license (MIT)
- Generic vss robot model
- Gazebo camera plugin
- File requirements.txt for python virtual environments

### Changed

- Upgrade to Gazebo 11 and ROS noetic
- Simulation environment released as free software

### Deprecated

- Remove original model of ThunderVolt team

### Known problems

- Ball physical properties
- The simulation fails to start sometimes

## [20.05.2]

### Added

- Simulation of dry and viscous friction in wheels movement
- Dependency ```effort_controllers```

### Changed

- Topics receive the command of **torque in Nm**
  - /robotX/vss_robot_left_controller/command
  - /robotX/vss_robot_right_controller/command
- Name standardization with VSSVision

### Known problems

- Ball physical properties
- The simulation fails to start sometimes

## [20.05.1] - Iron Cup 2020

### Added

- Mechanical project v1.1 - Thalles e Diego
- Motors control with **velocity in rad/s**
  - /robot_X/vss_robot_left_controller/command
  - /robot_X/vss_robot_right_controller/command
- Simulation environments
  - Single robot
  - Team of 3
  - Match 3x3
- Dependency ```velocity_controllers```

### Known problems

- Ball physical properties
- Motors behavior too ideal - far from reality

[21.04.1]: https://github.com/ThundeRatz/travesim/releases/tag/v21.04.1
[21.03.2]: https://github.com/ThundeRatz/travesim/releases/tag/v21.03.2
[21.03.1]: https://github.com/ThundeRatz/travesim/releases/tag/v21.03.1
[20.10.1]: https://github.com/ThundeRatz/travesim/releases/tag/v20.10.1
[20.05.2]: https://github.com/ThundeRatz/travesim/releases/tag/v20.05.2
[20.05.1]: https://github.com/ThundeRatz/travesim/releases/tag/v20.05.1
