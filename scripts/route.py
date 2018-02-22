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

def search_for_door_exit():
    print 'searching for dor exit'
    waits(Rotate(30, laser_threshold=0.5, threshold_direction=False, laser_width=0.2, range_center=False))

def align_wheels(angle):
   print 'aligning wheels'
   waits(Rotate(angle, laser_threshold=0, threshold_direction=False, laser_width=0.45))
    
def turn_towards_hall():
   print 'turning towards hall'
   waits(Rotate(350, laser_threshold=5, threshold_direction=False, laser_width=0.45))
     
def pass_door():
    print 'in pass_door'
    waits(MovementManager(3, obstacle_threshold=1.6, laser_width=0.45))


def move_down_hall():
    print 'moving down hall'
    for i in range(8):
        waits(MovementManager(10, obstacle_threshold=0.5, laser_width=0.45))   
        
        
	


def main():
    rospy.init_node('route_node')
    """
    reach_door()
    turn_to_door()
    align_wheels(-2)
    search_for_door_exit()
    align_wheels(-2)
    pass_door()
    turn_towards_hall()
    align_wheels(-2)
    """
    #move_down_hall()
    #rospy.loginfo("right : {}  left : {}".format())
    

if __name__ == "__main__":
    main()
