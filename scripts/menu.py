#! /usr/bin/python2
from __future__ import print_function
import rospy
from os import system
from MovementManager import MovementManager
from object_cords_handler import ObjectCordsHandler
from rotate import Rotate
from cam import ObjectDetector
from move_to_red import MoveToRedObject
from arm import ArmController

clear = lambda: system('clear')

def get_angle():
    angle = raw_input("\tEnter angle (in degrees) for rotation: ")
    angle = int(angle.strip())
    angle %= 360
    if angle > 180:
        angle -= 360
    return angle

def prompt():
    print("[1] Move forward")
    print("[2] Turn around")
    print("[3] Distance to red object")
    print("[4] move toward red object")
    print("[5] move arm to red object")

    return raw_input('\nEnter your choice: ')

def object_detector():
    obj = ObjectDetector()
    res = obj.get_distance_from_red_object()
    obj.wait_for_cb()
    if res is not None:
        res = res[0]/1000.0
    print(res)

def move_arm():
    objectCordsHandler = ObjectCordsHandler()
    #move_arm_forward = lambda: ArmController().move_arm_forward()
    ArmController().move_arm_forward()

def move_to_red_f():
    for i in xrange(20):
	# not goo
	MoveToRedObject()

def main():
    rospy.init_node('main_node')
    waits = lambda k: k.wait_for_cb()

    move = lambda : waits(MovementManager(0.5))
    rotate = lambda: waits(Rotate(get_angle()))

    move_to_red = lambda: waits(MoveToRedObject())

    funcs = [move, rotate, object_detector, move_to_red, move_arm]

    stop = False
    while not stop:
        inp = prompt()
        try:
            clear()
            choice = int(inp) - 1
            (funcs[choice]())
            continue

        except ValueError:
            if inp == 'q':
                stop = True
                continue

        except IndexError:
            pass

        print('Invalid choice. try again')


if __name__ == "__main__":
    main()
