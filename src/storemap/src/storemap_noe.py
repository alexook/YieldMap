#!/usr/bin/env python
import rospy
import sys, select, tty, termios, signal
from std_msgs.msg import String, Bool

pub = rospy.Publisher('/my_topic', String, queue_size=1)
switch_pub = rospy.Publisher('/map_switch', Bool, queue_size=1)
old_attr = termios.tcgetattr(sys.stdin)

def storemap():

    bool_msg = Bool()
    bool_msg.data = True
    switch_pub.publish(bool_msg)

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    print('Restore the termios done')
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('stormap_node', anonymous=True)


    signal.signal(signal.SIGINT, signal_handler)
    rate = rospy.Rate(100)
    
    tty.setcbreak(sys.stdin.fileno())
    print('Please input keys, press Ctrl-C to quit')
    while not rospy.is_shutdown():
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            r = sys.stdin.read(1)
            if r == "s":
                print(r)
                storemap()
            else:
                pub.publish(r)

        rate.sleep()
    



