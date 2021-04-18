#!/usr/bin/env python3
# coding=utf-8
"""
    File:
        keyboard_node.py

    Description:
        Simple python routine to watch the keyboard or a joystick
        to send velocity commands to a Gazebo simulation.
"""

import pygame
import sys

import rospy
from geometry_msgs.msg import Twist

# Vamos acompanhar o estado dessas teclas
KEYS = [pygame.K_w, pygame.K_a, pygame.K_s, pygame.K_d]

# Indice dos eixos x e y do joystick
X_AXIS = 0
Y_AXIS = 4
INVERT_X_AXIS = True
INVERT_Y_AXIS = True

ROBOTS = 3

# Namespace dos tópicos que iremos publicar
DEFAULT_NAMESPACE = "/yellow_team/robot_{}"

DEFAULT_DEBUG = False

# A vel máxima do robô é 2 m/s
MAX_LIN_VEL = 1.0  # m/s

# A vel máxima do robô é 40 rad/s
MAX_ROT_VEL = 20  # rad/s

# Define a rampa de aceleração quando usando o teclado
# Valores em porcentagem da velocidade máxima
KEYBOARD_LINEAR_STEP = 0.03
KEYBOARD_LINEAR_MAX = 1.0

KEYBOARD_ANGULAR_STEP = 0.03
KEYBOARD_ANGULAR_MAX = 0.6

# Os comandos vão de -126 até 126 de modo que os bytes 0xFE e 0xFF
# nunca são utilizados
SCALE = 126


def getNamespace(number):
    return DEFAULT_NAMESPACE.format(number)


def drawConsole(win, font, console):
    """
    Fills window console with the sentences stored in the list console
        :param win: pygame.display Window object to be filled
        :param font: pygame.Font Font style to be used
        :param console: list<font.render> List of text to write
    """
    img = font.render("Event console Area", 1, (155, 155, 155), (0, 0, 0))
    win.blit(img, (2, 132))
    ypos = 450
    h = list(console)
    h.reverse()
    for line in h:
        r = win.blit(line, (10, ypos))
        win.fill(0, (r.right, r.top, 620, r.height))
        ypos -= font.get_height()


def main(debug=DEFAULT_DEBUG):

    vel_pub = []
    rate = None
    current_robot = 0

    # Inicia configs do ROS
    rospy.init_node('vss_human_controller')

    for i in range(ROBOTS):
        vel_pub.append(rospy.Publisher(
            getNamespace(i) + '/diff_drive_controller/cmd_vel',
            Twist, queue_size=2))

    rate = rospy.Rate(60)  # 60hz

    pygame.init()

    # Cria a janela
    win = pygame.display.set_mode((640, 480), pygame.RESIZABLE)
    pygame.display.set_caption("Keyboard Comunication Interface")

    # Lista de frases a serem mostradas no console
    console = []
    font = pygame.font.Font(None, 26)

    # Dicionário para guardar o estado de algumas teclas
    state = {}
    for key in KEYS:
        state[key] = False

    # Vamos detectar os Joysticks conectados ao computador
    axis = [0.0, 0.0]
    using_joystick = True
    for x in range(pygame.joystick.get_count()):
        j = pygame.joystick.Joystick(x)
        j.init()
        txt = "Enabled joystick: " + j.get_name()
        print(txt)
        img = font.render(txt, 1, (50, 200, 50), (0, 0, 0))
        console.append(img)

    if not pygame.joystick.get_count():
        using_joystick = False
        print("No Joysticks to Initialize")
        img = font.render("No Joysticks to Initialize", 1,
                          (50, 200, 50), (0, 0, 0))
        console.append(img)

    vel_lin = 0.0
    vel_ang = 0.0

    running = True
    while running:

        for e in pygame.event.get():

            # Movimento dos botões do teclado
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    running = False
                if e.key in KEYS:
                    state[e.key] = True
            elif e.type == pygame.KEYUP:
                if e.key in KEYS:
                    state[e.key] = False

            # Movimento dos direcionais do joystick
            if e.type == pygame.JOYAXISMOTION:
                if e.dict['axis'] in (X_AXIS, Y_AXIS):
                    if e.dict['axis'] == X_AXIS:
                        if INVERT_X_AXIS:
                            axis[0] = -e.value
                        else:
                            axis[0] = e.value

                    elif e.dict['axis'] == Y_AXIS:
                        if INVERT_Y_AXIS:
                            axis[1] = -e.value
                        else:
                            axis[1] = e.value

            # Caso algum botão do joystick seja apertado
            if e.type == pygame.JOYBUTTONDOWN \
                or e.type == pygame.JOYBUTTONUP \
                    or e.type == pygame.JOYHATMOTION:

                txt = "%s: %s" % (pygame.event.event_name(e.type), e.dict)
                print(txt)
                img = font.render(txt, 1, (50, 200, 50), (0, 0, 0))
                console.append(img)
                console = console[-13:]

            # L1 pressionado
            if (e.type == pygame.JOYBUTTONDOWN and e.dict['button'] == 4) or (e.type == pygame.KEYDOWN and e.key == pygame.K_e):
                current_robot += 1
                current_robot %= ROBOTS

            # R1 pressionado
            if (e.type == pygame.JOYBUTTONDOWN and e.dict['button'] == 5) or (e.type == pygame.KEYDOWN and e.key == pygame.K_q):
                current_robot -= 1
                current_robot %= ROBOTS

            elif e.type == pygame.VIDEORESIZE:
                win = pygame.display.set_mode(e.size, pygame.RESIZABLE)

            elif e.type == pygame.QUIT:
                running = False

        drawConsole(win, font, console)
        pygame.display.flip()

        if using_joystick:
            txt = f"Linear: {int(axis[1]*SCALE)} Angular: {int(axis[0]*SCALE)}"
            img = font.render(txt, 1, (50, 200, 50), (0, 0, 0))
            console.append(img)
            console = console[-13:]

            if debug:
                print(txt)

            vel_cmd_twist = Twist()
            vel_cmd_twist.linear.x = axis[1]*MAX_LIN_VEL
            vel_cmd_twist.angular.z = axis[0]*MAX_ROT_VEL

            vel_pub[current_robot].publish(vel_cmd_twist)

        else:
            if state[pygame.K_w] and not state[pygame.K_s]:
                vel_lin += KEYBOARD_LINEAR_STEP
                vel_lin = min(vel_lin, KEYBOARD_LINEAR_MAX)
            elif state[pygame.K_s] and not state[pygame.K_w]:
                vel_lin -= KEYBOARD_LINEAR_STEP
                vel_lin = max(vel_lin, -KEYBOARD_LINEAR_MAX)
            else:
                vel_lin = 0.0

            if state[pygame.K_a] and not state[pygame.K_d]:
                vel_ang += KEYBOARD_ANGULAR_STEP
                vel_ang = min(vel_ang, KEYBOARD_ANGULAR_MAX)
            elif state[pygame.K_d] and not state[pygame.K_a]:
                vel_ang -= KEYBOARD_ANGULAR_STEP
                vel_ang = max(vel_ang, -KEYBOARD_ANGULAR_MAX)
            else:
                vel_ang = 0.0

            txt = f"Linear: {int(vel_lin*SCALE)} Angular: {int(vel_ang*SCALE)}"
            img = font.render(txt, 1, (50, 200, 50), (0, 0, 0))
            console.append(img)
            console = console[-13:]

            if debug:
                print(txt)

            vel_cmd_twist = Twist()
            vel_cmd_twist.linear.x = vel_lin * MAX_LIN_VEL
            vel_cmd_twist.angular.z = vel_ang * MAX_ROT_VEL

            vel_pub[current_robot].publish(vel_cmd_twist)

        rate.sleep()


if __name__ == "__main__":
    rospy.loginfo("Começando a brincadeira!")

    # Clean ROS parameters from command line
    myargv = rospy.myargv(argv=sys.argv)

    print(myargv)
    rospy.loginfo(myargv)

    main()
