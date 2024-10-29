#!/usr/bin/env python

import rospy
from std_srvs.srv import order, orderResponse
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def navigate_to(position, move_base_client):
    """Naviga verso una posizione specificata."""
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    goal.target_pose.pose.position.x = position['x']
    goal.target_pose.pose.position.y = position['y']
    goal.target_pose.pose.orientation.w = 1.0  # Assume orientamento in avanti
    
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()
    
    # Restituisce True se l'obiettivo è stato raggiunto, False altrimenti
    return move_base_client.get_result() is not None

def handle_order(req, status_pub, move_base_client, counter_pos, table_pos):
    """Gestisce la richiesta di un ordine."""
    # Fase 1: Naviga al bancone
    status_pub.publish("Navigating to the counter")
    if not navigate_to(counter_pos, move_base_client):
        return orderResponse(success=False, message="Failed to reach counter")
    
    # Fase 2: Naviga al tavolo
    status_pub.publish(f"Navigating to table")
    if not navigate_to(table_pos, move_base_client):
        return orderResponse(success=False, message="Failed to reach table")
    
    # Aggiornamento stato finale
    status_pub.publish("Order delivered successfully")
    return orderResponse(success=True, message="Order delivered")

def waiter_node():
    """Inizializza il nodo waiter e i suoi componenti."""
    rospy.init_node("waiter_node")
    
    # Inizializza il publisher per lo stato di consegna
    status_pub = rospy.Publisher("/delivery_status", String, queue_size=10)
    
    # Inizializza l'action client per la navigazione
    move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    move_base_client.wait_for_server()
    
    # Carica le posizioni dal Parameter Server
    counter_pos = rospy.get_param("bancone_pos")
    table_positions = rospy.get_param("table_positions")
    table_id = "table_1"  # Tavolo del cliente (fisso per ora)
    table_pos = table_positions.get(table_id)

    # Definisce il wrapper per passare gli argomenti al servizio
    def handle_order_wrapper(req):
        return handle_order(req, status_pub, move_base_client, counter_pos, table_pos)

    # Inizializza il servizio per ricevere ordini
    rospy.Service("/order_drink", order, handle_order_wrapper)
    
    rospy.loginfo("Waiter node is ready to receive orders.")
    rospy.spin()

if __name__ == "__main__":
    waiter_node()
