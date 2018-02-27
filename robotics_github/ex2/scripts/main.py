from __future__ import print_function
import rospy
from os import system
from MovementManager import MovementManager
from rotate import Rotate
from cam import ObjectDetector
from move_to_red import MoveToRedObject
from arm import ArmController

clear = lambda: system('clear')

def get_angle():
    angle = raw_input("\tEnter angle (in degrees) for rotation: ")
    return int(angle.strip())

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
    print(res)

def main():
    rospy.init_node('main_node')
    waits = lambda k: k.wait_for_cb()

    move = lambda : waits(MovementManager(0.5))
    rotate = lambda: waits(Rotate(get_angle()))

    move_to_red = lambda: waits(MoveToRedObject())
    move_arm_forward = lambda: ArmController().move_arm_forward()

    funcs = [move, rotate, object_detector, move_to_red, move_arm_forward]

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
