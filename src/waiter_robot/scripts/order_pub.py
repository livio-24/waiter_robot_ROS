#!/usr/bin/env python
import sys
import random
import rospy
from waiter_robot.msg import order_msg, menu_item
from waiter_robot.srv import menu
    
def publish_order(table_id, menu_items):

    # Create pub
    pub = rospy.Publisher('/order_topic', order_msg, queue_size=10)
    rospy.sleep(1)

    ordermsg = order_msg()
    ordermsg.table_id = table_id
    ordermsg.items = [menu_item(name=item['name'], price=item['price']) for item in menu_items]

    rospy.loginfo(f"Publishing order for {ordermsg.table_id}: {menu_items}")
    
    pub.publish(ordermsg)

if __name__ == '__main__':
    rospy.init_node(f'tables_orders_publisher', anonymous=True)
    rospy.spin()
    #try:
    #    publish_order(str(sys.argv[1]))
    #except rospy.ROSInterruptException:
    #    pass
