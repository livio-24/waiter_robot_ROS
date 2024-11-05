#!/usr/bin/env python
import sys
import random
import rospy
from waiter_robot.msg import order_msg

def single_order_publisher(table_id, order_details):
    # Inizializza il nodo publisher
    rospy.init_node('order_publisher', anonymous=True)
    
    # Crea il publisher per il topic 'order_topic' e il tipo di messaggio 'Order'
    pub = rospy.Publisher('/order_topic', order_msg, queue_size=10)
    rospy.sleep(1)

    # Crea un singolo messaggio di tipo 'Order'
    ordermsg = order_msg()
    ordermsg.table_id = table_id
    ordermsg.order_details = order_details

    # Log per confermare l'invio
    rospy.loginfo(f"Pubblico ordine per il tavolo {ordermsg.table_id}: {ordermsg.order_details}")
    
    # Pubblica il messaggio una sola volta
    pub.publish(ordermsg)

    # Attendi un breve periodo per garantire che il messaggio sia trasmesso
    rospy.sleep(1)
    rospy.loginfo("Ordine pubblicato e nodo terminato.")

if __name__ == '__main__':
    beverage_options = ["birra", "caffè", "coca-cola"]
    try:
        single_order_publisher(str(sys.argv[1]), random.choice(beverage_options))
    except rospy.ROSInterruptException:
        pass
