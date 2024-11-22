#!/usr/bin/env python3
import rospy
from waiter_robot.msg import order_msg, menu_item

def order_callback(msg):
    # Recupera gli ordini attuali dal parameter server
    orders = rospy.get_param('/orders', {})
    rospy.loginfo(orders)
    # Prepara l'ordine per il tavolo specifico
    if msg.table_id not in orders:
        orders[msg.table_id] = []
    
    for item in msg.items:
        orders[msg.table_id].append({"nome": item.nome, "prezzo": item.prezzo})
    
    # Salva gli ordini aggiornati nel parameter server
    rospy.set_param('/orders', orders)
    #rospy.loginfo(f"Ordine ricevuto per il tavolo {msg.table_id}: {msg.items}")

def order_listener():
    rospy.init_node('orders_saver')
    rospy.Subscriber('/order_topic', order_msg, order_callback)
    #rospy.loginfo("Nodo 'order_listener' in ascolto sul topic '/new_order'.")
    rospy.spin()

if __name__ == "__main__":
    order_listener()
