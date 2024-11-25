#!/usr/bin/env python3
import rospy
from waiter_robot.srv import request_bill, request_billResponse
from waiter_robot.msg import menu_item


def handle_request_bill(req):
    # Get orders from parameter server
    orders = rospy.get_param('/orders', {})

    # check if table exists in active orders
    if req.table_id not in orders:
        return request_billResponse(
            success=False,
            total=0.0,
            items=[],
            message="Tavolo non trovato."
        )
    
    table_orders = orders[req.table_id]
    total = 2.0 #coperto
    items_list = []

    # Calculate the total
    for item_data in table_orders:
        item = menu_item()
        item.name = item_data['name']
        item.price = item_data['price']
        items_list.append(item)
        total += item.price

    del orders[req.table_id]  # remove table and relative orders from server
    rospy.set_param('/orders', orders) 
    #send response
    return request_billResponse(
        success=True,
        total=total,
        items=items_list,
        message="Thanks for coming."
    )

def bill_server():
    rospy.init_node('bill_service_server')
    rospy.Service('/request_bill', request_bill, handle_request_bill)
    rospy.loginfo("Bill service is ready.")
    rospy.spin()

if __name__ == "__main__":
    try:
        bill_server()
    except rospy.ROSInterruptException:
        pass
