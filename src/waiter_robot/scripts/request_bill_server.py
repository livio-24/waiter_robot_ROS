#!/usr/bin/env python3
import rospy
from waiter_robot.srv import request_bill, request_billResponse
from waiter_robot.msg import menu_item


def handle_request_bill(req):
    # Recupera gli ordini dal parameter server
    orders = rospy.get_param('/orders', {})

    # Verifica se il tavolo esiste
    if req.table_id not in orders:
        return request_billResponse(
            success=False,
            total=0.0,
            items=[],
            message="Tavolo non trovato."
        )
    
    # Recupera gli articoli dell'ordine
    table_orders = orders[req.table_id]
    total = 0.0
    items_list = []

    # Calcola il totale
    for item_data in table_orders:
        item = menu_item()
        item.nome = item_data['nome']
        item.prezzo = item_data['prezzo']
        items_list.append(item)
        total += item.prezzo

    del orders[req.table_id]  # Rimuove il tavolo e i relativi ordini
    rospy.set_param('/orders', orders) 
    # Ritorna la risposta
    return request_billResponse(
        success=True,
        total=total,
        items=items_list,
        message="Conto calcolato con successo."
    )

def bill_server():
    rospy.init_node('bill_service_server')
    
    # Inizializza il servizio
    rospy.Service('/request_bill', request_bill, handle_request_bill)
    
    rospy.loginfo("Bill service is ready.")
    
    rospy.spin()

if __name__ == "__main__":
    try:
        bill_server()
    except rospy.ROSInterruptException:
        pass
