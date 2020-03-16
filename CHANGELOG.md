# Changelog

## v2.0.0-alpha

- Quebra de compatibilidade com a v1.0.0
- Tópicos recebem o valor do **torque em Nm**
  - /robotX/vss_robot_left_controller/command
  - /robotX/vss_robot_right_controller/command
- Padronização da nomenclatura com o VSSVision
- Simulação do atrito seco e viscoso no movimento das rodas
- Adição do ```effort_controllers``` como dependência

### Problemas conhecidos

- Propriedades físicas da bola
- A simulação falha em iniciar ocasionalmente

## v1.0.0 - Iron Cup 2020

- Primeira versão funcional
- Projeto mecânico v1.1 - Thalles e Diego
- Controle dos motores pela **velocidade em rad/s**
  - /robot_X/vss_robot_left_controller/command
  - /robot_X/vss_robot_right_controller/command
- Ambientes de simulação
  - Robô único
  - Time de 3
  - Partida 3x3
- Adição do ```velocity_controllers``` como dependência

### Problemas conhecidos

- Propriedades físicas da bola
- Comportamento dos motores ideal demais - longe da realidade
