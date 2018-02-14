import rospy
from os import system
from MovementManager import MovementManager
from rotate import Rotate

waits = lambda k: k.wait_for_cb()

def reach_door():
    print 'in reach_door'
    waits(MovementManager(3, obstacle_threshold=0.50))

def turn_to_door():
    print 'in turn_to_door'
    waits(Rotate(350, laser_threshold=2.8, threshold_direction=False, laser_width=0.45))

def pass_door():
    print 'in pass_door'
    waits(MovementManager(3, obstacle_threshold=1.6, laser_width=0.45))

def main():
    rospy.init_node('route_node')
    #reach_door()
    #turn_to_door()
    pass_door()


if __name__ == "__main__":
    main()
