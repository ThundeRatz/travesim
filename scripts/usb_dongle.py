#!/usr/bin/env python
# coding=utf-8
"""
    File:
        usb_dongle.py

    Description:
        Simple python routine to watch the keyboard or a joystick
    to send velocity commands to a Gazebo simulation
        See https://github.com/ThundeRatz/VSSFirmware
"""

import pygame
import argparse

import rospy
from geometry_msgs.msg import Twist
# Vamos acompanhar o estado dessas teclas
KEYS = [pygame.K_a, pygame.K_s, pygame.K_d, pygame.K_w]

# Indice dos eixos x e y do joystick
X_AXIS = 0
Y_AXIS = 4
INVERT_X_AXIS = False
INVERT_Y_AXIS = True
ANG_SCALE = 6
LIN_SCALE = 1

# Os comandos vão de -126 até 126 de modo que os bytes 0xFE e 0xFF
# nunca são utilizados
SCALE = 126

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


def main():

    vel_pub = None
    vel_msg = None
    rate = None

    # Inicia configs do ROS
    rospy.init_node('vss_human_controller')
    vel_pub = rospy.Publisher('/robot_0/vss_robot_diff_drive_controller/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    rate = rospy.Rate(10) # 10hz

    pygame.init()

    # Cria a janela
    win = pygame.display.set_mode((640, 480), pygame.RESIZABLE)
    pygame.display.set_caption("nRFDongle Comunication Interface")

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
        img = font.render("No Joysticks to Initialize", 1, (50, 200, 50), (0, 0, 0))
        console.append(img)

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

            elif e.type == pygame.VIDEORESIZE:
                win = pygame.display.set_mode(e.size, pygame.RESIZABLE)

            elif e.type == pygame.QUIT:
                running = False

        drawConsole(win, font, console)
        pygame.display.flip()

        if using_joystick:
            txt = "X: {} Y: {}".format(int(axis[0]*SCALE), int(axis[1]*SCALE))
            print(txt)
            img = font.render(txt, 1, (50, 200, 50), (0, 0, 0))
            console.append(img)
            console = console[-13:]

            vel_msg.linear.x = axis[1]*LIN_SCALE
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = -axis[0]*ANG_SCALE

        else:
            vel_y = 0.0
            if state[pygame.K_a]:
                vel_y += 1.0
            if state[pygame.K_d]:
                vel_y -= 1.0

            vel_x = 0.0
            if state[pygame.K_s]:
                vel_x -= 1.0
            if state[pygame.K_w]:
                vel_x += 1.0

            txt = "X: {} Y: {}".format(int(vel_x*SCALE), int(vel_y*SCALE))
            print(txt)
            img = font.render(txt, 1, (50, 200, 50), (0, 0, 0))
            console.append(img)
            console = console[-13:]

            vel_msg.linear.x = vel_x*LIN_SCALE
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = vel_y*ANG_SCALE

        vel_pub.publish(vel_msg)
        rate.sleep()


if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # args = parser.parse_args()

    main()
