#!/usr/bin/env python

import rospy
from utils.msg import KeyboardCommand

import math
import numpy
import sys, select, termios, tty

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

speed = 0.8
turn = 1.0
target_speed = 0
target_turn = 0
control_speed = 0
control_turn = 0
enabled = 1.0

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def onKeyLoop(event):
    global target_speed, target_turn, control_speed, control_turn

    if target_speed > control_speed:
        control_speed = min( target_speed, control_speed + 0.05 )
    elif target_speed < control_speed:
        control_speed = max( target_speed, control_speed - 0.05 )
    else:
        control_speed = target_speed

    if target_turn > control_turn:
        control_turn = min( target_turn, control_turn + 0.05 )
    elif target_turn < control_turn:
        control_turn = max( target_turn, control_turn - 0.05 )
    else:
        control_turn = target_turn

    cmd_msg = KeyboardCommand()
    cmd_msg.header.stamp = event.current_real
    cmd_msg.mode = enabled
    cmd_msg.forward_pwm = control_speed
    cmd_msg.side_pwm = -control_turn
    pub.publish(cmd_msg)

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('thrust_commander')
    pub = rospy.Publisher('thrust/keyboard_cmd', KeyboardCommand, queue_size=5)
    rospy.Timer(rospy.Duration(0.15), onKeyLoop)
    rospy

    x = 0
    th = 0
    status = 0
    count = 0
    print('Keyboard control enabled? {}'.format(enabled))

    print vels(speed,turn)
    while(1):
        key = getKey()

        if key in moveBindings.keys():
            x = moveBindings[key][0]
            th = moveBindings[key][1]
            count = 0

        elif key in speedBindings.keys():
            speed = speed * speedBindings[key][0]
            turn = turn * speedBindings[key][1]
            count = 0

            print vels(speed,turn)
            if (status == 14):
                print msg
            status = (status + 1) % 15

        elif key == 'k':
            x = 0
            th = 0
            control_speed = 0
            control_turn = 0

        elif key == ' ':
            if enabled == 1.0:
                enabled = 0.0
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                enabled = 1.0
            print('Keyboard control enabled? {}'.format(enabled))
        else:
            count = count + 1
            if count > 4:
                x = 0
                th = 0
            if (key == '\x03'):
                break

        target_speed = speed * x
        target_turn = turn * th

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

