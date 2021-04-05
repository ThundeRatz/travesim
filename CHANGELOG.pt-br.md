# Changelog

O formato é baseado no [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
e esse projeto segue a convenção [versionamento semântico](https://semver.org/spec/v2.0.0.html).

## [21.04.1] - 2021-04-05

### Modificado

- Nome do projeto passa a ser **TraveSim** ao invés de **vss_simulation**
- Projeto passa a utilizar o versionamento cronológico no formato YY.0D.MINOR

## [21.03.2] - 2021-03-26

### Modificado

- Nomes dos robôs agora seguem a convenção /[yellow|blue]\_team/robot\_[0..2]. Essa mudança se aplica aos nomes dos modelos dos robôs e ao namespace dos tópicos
- O tópico dos controladores mudou da forma
  - **/[yellow|blue]\_team/robot\_[0..2]/~~vss_robot~~\_diff\_drive\_controller/cmd\_vel** - **/[yellow|blue]\_team/robot\_[0..2]/diff\_drive\_controller/cmd\_vel**
  - **/[yellow|blue]\_team/robot\_[0..2]/~~vss_robot~~\_[left|right]_controller/command** - **/[yellow|blue]\_team/robot\_[0..2]/[left|right]_controller/command**

## [21.03.1] - 2021-03-22

### Adicionado

- Interface de controle selecionável: Controlador diferencial (mensagens Twist) ou controle direto por velocidade angular das rodas

### Modificado

- Migração do script de controle com teclado

### Obsoleto

- Controle direto das rodas por torque

## [20.10.1] - 2020-10-05 - [Open Camera]

### Adicionado

- Licensa de software (MIT)
- Modelo de robô vss genérico
- Plugin de câmero do Gazebo
- Arquivo requirements.txt para ambientes virtuais de python

### Modificado

- Atualização para Gazebo 11 e ROS noetic
- Ambiente de simulação liberado como software livre

### Obsoleto

- Remoção do modelo original do time ThunderVolt

### Problemas conhecidos

- Propriedades físicas da bola
- A simulação falha em iniciar ocasionalmente

## [20.05.2] - 2020-05-12

### Adicionado

- Simulação de atrito seco e viscoso no movimento das rodas
- Dependência ```effort_controllers```

### Modificado

- Tópicos recebem o valor do **torque em Nm**
  - /robotX/vss_robot_left_controller/command
  - /robotX/vss_robot_right_controller/command
- Padronização da nomenclatura com o VSSVision
- Simulação do atrito seco e viscoso no movimento das rodas

### Problemas conhecidos

- Propriedades físicas da bola
- A simulação falha em iniciar ocasionalmente

## [20.05.1] - 2020-05-04 - [Iron Cup 2020]

### Adicionado

- Projeto mecânico v1.1 - Thalles e Diego
- Controle dos motores pela **velocidade em rad/s**
  - /robot_X/vss_robot_left_controller/command
  - /robot_X/vss_robot_right_controller/command
- Ambientes de simulação
  - Robô único
  - Time de 3
  - Partida 3x3
- Dependência```velocity_controllers```

### Problemas conhecidos

- Propriedades físicas da bola
- Comportamento dos motores ideal demais - longe da realidade

[Open Camera]: https://github.com/ThundeRatz/travesim/releases/tag/v20.10.1
[Iron Cup 2020]: https://github.com/ThundeRatz/travesim/releases/tag/v20.05.1
[21.04.1]: https://github.com/ThundeRatz/travesim/releases/tag/v21.04.1
[21.03.2]: https://github.com/ThundeRatz/travesim/releases/tag/v21.03.2
[21.03.1]: https://github.com/ThundeRatz/travesim/releases/tag/v21.03.1
[20.10.1]: https://github.com/ThundeRatz/travesim/releases/tag/v20.10.1
[20.05.2]: https://github.com/ThundeRatz/travesim/releases/tag/v20.05.2
[20.05.1]: https://github.com/ThundeRatz/travesim/releases/tag/v20.05.1
