#!/usr/bin/env python
import sys
import random
import rospy
from waiter_robot.msg import order_msg, menu_item
from waiter_robot.srv import menu
    
def publish_order(table_id, menu_items):

    # Crea il publisher per il topic 'order_topic' e il tipo di messaggio 'order_msg'
    pub = rospy.Publisher('/order_topic', order_msg, queue_size=10)
    rospy.sleep(1)

    ordermsg = order_msg()
    ordermsg.table_id = table_id
    ordermsg.items = [menu_item(nome=item['nome'], prezzo=item['prezzo']) for item in menu_items]
    #ordermsg.quantities = quantities

    # Log per confermare l'invio
    rospy.loginfo(f"Publishing order for {ordermsg.table_id}: {menu_items}")
    
    pub.publish(ordermsg)

if __name__ == '__main__':
        # Inizializza il nodo publisher
    rospy.init_node(f'tables_orders_publisher', anonymous=True)
    rospy.spin()
    #try:
    #    publish_order(str(sys.argv[1]))
    #except rospy.ROSInterruptException:
    #    pass
