from time import sleep
from rospy import Subscriber

class Waitable(object):

    def __init__(self, wait_time=0.2):
        subscribers = [getattr(self, x) \
                            for x in dir(self) if type(getattr(self, x)) is Subscriber]
        self.subscribers = [(x, x.impl.__sizeof__) for x in subscribers]
        self.wait_time = wait_time

    def wait_for_cb(self):
        # nasty trick
        while any([x.impl.__sizeof__ == sz for x, sz in self.subscribers]):
            sleep(self.wait_time)

    def force_close(self):
        for sub, __ in self.subscribers:
            sub.unregister()
