#!/usr/bin/env python

import rospy
from waiter_robot.msg import order_msg
from navigation import send_goal

orders = []


def order_callback(msg):
    global orders

    if(msg.table_id == 'delivery'):
        #check if orders not empty
        if(orders):
            deliver()
            return
        
        else:
            return
    
    # Create a new order
    order = {"table_id": msg.table_id, 
              "items": msg.items
            }

    orders.append(order)
    rospy.loginfo(f"Riceived order from {order['table_id']}: {order['items']}")

def deliver():
    global orders

    rospy.loginfo("Navigating to kitchen.")
    send_goal('kitchen')  # GO kitchen

    # Send robot to each table in the order list
    while orders:
        order = orders.pop(0)  # take first order in the list
        rospy.loginfo(f"going to table {order['table_id']} to deliver: {order['items']}")
        send_goal(order["table_id"])
        rospy.loginfo(f"order deliverd to {order['table_id']}.")

    orders = []
 


def order_subscriber():
    # Init node
    rospy.init_node('ordine_subscriber', anonymous=True)

    # Create sub 
    rospy.Subscriber('order_topic', order_msg, order_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        order_subscriber()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"Eccezione ROS: {e}")
