#!/usr/bin/env python
# coding=utf-8
"""
    File:
        usb_dongle.py

    Description:
        Simple python routine to watch the keyboard or a joystick
    to send velocity commands to a connected nRF Dongle

    Arguments:
        -s, --serial-port       Serial port of nRF Dongle
        -b, --baudrate          Baudrate for serial comunication

    Protocol:
        +------+-------+-------+-------+------+
        | 0xFF | Vel 1 | Vel 2 | CRC-8 | 0xFE |
        +------+-------+-------+-------+------+

            0xFF - Start byte
            Vel 1 - signed byte from -126 to 126
            Vel 2 - signed byte from -126 to 126
            CRC-8 - 8 bits CRC from 0x1AB generator
            0xFE - Stop byte
"""

# from typing import List
from ctypes import c_byte
import numpy as np
import serial
import pygame
import time
import crcmod
import argparse

HAS_ROS = True
import rospy
try:
    import rospy
    from geometry_msgs.msg import Twist
except ModuleNotFoundError:
    HAS_ROS = False

print("HAS_ROS: {}".format(HAS_ROS))

# Vamos acompanhar o estado dessas teclas
KEYS = [pygame.K_a, pygame.K_s, pygame.K_d, pygame.K_w]

# Indice dos eixos x e y do joystick
X_AXIS = 0
Y_AXIS = 4
INVERT_X_AXIS = False
INVERT_Y_AXIS = True
ANG_SCALE = 6
LIN_SCALE = 1

DEFAULT_SERIAL_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 9600

# Os comandos vão de -126 até 126 de modo que os bytes 0xFE e 0xFF
# nunca são utilizados
SCALE = 126

crcCalc = crcmod.mkCrcFun(0x1AB, initCrc=0, rev=False)


def drawConsole(win, font,
                console):
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


def main(serial_port = DEFAULT_SERIAL_PORT,
         baudreate = DEFAULT_BAUDRATE):

    vel_pub = None
    vel_msg = None
    rate = None
    if HAS_ROS:
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
            sendCommand(axis[0], axis[1], serial_port, baudreate)
            # txt = f"X: {int(axis[0]*SCALE)} Y: {int(axis[1]*SCALE)}"
            txt = "X: {} Y: {}".format(int(axis[0]*SCALE), int(axis[1]*SCALE))
            print(txt)
            img = font.render(txt, 1, (50, 200, 50), (0, 0, 0))
            console.append(img)
            console = console[-13:]

            if HAS_ROS:
                vel_msg.linear.x = axis[1]*LIN_SCALE
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = -axis[0]*ANG_SCALE

        else:
            vel_y = 0.0
            if state[pygame.K_a]:
                vel_y -= 1.0
            if state[pygame.K_d]:
                vel_y += 1.0

            vel_x = 0.0
            if state[pygame.K_s]:
                vel_x -= 1.0
            if state[pygame.K_w]:
                vel_x += 1.0

            sendCommand(vel_x, vel_y, serial_port, baudreate)
            # txt = f"X: {int(vel_x*SCALE)} Y: {int(vel_y*SCALE)}"
            txt = "X: {} Y: {}".format(int(vel_x*SCALE), int(vel_y*SCALE))
            print(txt)
            img = font.render(txt, 1, (50, 200, 50), (0, 0, 0))
            console.append(img)
            console = console[-13:]

            if HAS_ROS:
                vel_msg.linear.x = vel_x*LIN_SCALE
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = vel_y*ANG_SCALE

        if HAS_ROS:
            vel_pub.publish(vel_msg)
            rate.sleep()
        else:
            time.sleep(0.1)


def sendCommand(vel_x, vel_y,
                serial_port = DEFAULT_SERIAL_PORT,
                baudreate = DEFAULT_BAUDRATE):
    """
    Sends a velocity command through the serial
        :param vel_x: float from -1.0 to 1.0
        :param vel_y: float from -1.0 to 1.0
    """

    if abs(vel_x) > 1.0:
        vel_x = vel_x/abs(vel_x)

    if abs(vel_y) > 1.0:
        vel_y = vel_y/abs(vel_y)

    # SCALE = 126
    vel_x = int(SCALE*vel_x)
    vel_y = int(SCALE*vel_y)

    motor_dir = bytes(c_byte((vel_x + vel_y)//2))
    motor_esq = bytes(c_byte((vel_x - vel_y)//2))

    crc = bytes(crcCalc(motor_esq + motor_dir))
    msg = b'0xFF' + motor_esq + motor_dir + crc + b'0xFE'

    try:
        with serial.Serial(serial_port, baudreate) as dongle:
            dongle.write(msg)
    except serial.serialutil.SerialException:
        # print(f"Erro no serial! {serial_port} {baudreate}")
        print("Erro no serial! {} {}".format(serial_port, baudreate))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--serial-port",
                        help="Serial port of nRF Dongle", type=str,
                        default=DEFAULT_SERIAL_PORT)
    parser.add_argument("-b", "--baudrate",
                        help="Baudrate for serial communication", type=int,
                        default=DEFAULT_BAUDRATE)

    args = parser.parse_args()

    serial_port = args.serial_port
    baudrate = args.baudrate

    main(serial_port, baudrate)
