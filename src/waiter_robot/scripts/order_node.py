#!/usr/bin/env python

import rospy
from waiter_robot.srv import order
import random
import sys

def place_order(drinks, table_id):
    rospy.wait_for_service('/order')
    try:
        order_drink = rospy.ServiceProxy('/order_drink', table_id)
        drink = random.choice(drinks)
        rospy.loginfo(f"Placing order for {drink} for {table_id}")
        response = order_drink(drink, table_id)
        
        if response.accepted:
            rospy.loginfo(f"Order accepted: {response.message}")
        else:
            rospy.logwarn(f"Order rejected: {response.message}")
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


if __name__ == '__main__':
    rospy.init_node('order_node')
    drinks = ["Coffee", "Tea", "Juice", "Soda"]
    try:
        place_order(drinks, int(sys.argv[1]) )
    except rospy.ROSInterruptException:
        pass
