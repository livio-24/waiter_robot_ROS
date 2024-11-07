#!/usr/bin/env python
import sys
import random
import rospy
from waiter_robot.msg import order_msg
from waiter_robot.srv import menu

#service client
def request_menu():
    rospy.wait_for_service('get_menu')
    try:
        # Call the /get_menu service
        get_menu = rospy.ServiceProxy('get_menu', menu)
        response = get_menu()
        return response.item_names
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to get menu: {e}")
        return None, None
    
def create_random_order(menu_items, max_total_quantity=4):
    """Creates a random order with quantities summing up to max_total_quantity."""
    order_items = []
    order_quantities = []
    remaining_quantity = max_total_quantity
    
    # Randomly shuffle items
    random.shuffle(menu_items)

    # Iterate over items, assigning random quantities up to remaining_quantity
    for item in menu_items:
        if remaining_quantity == 0:
            break
        # Random quantity for the current item, ensuring it doesn't exceed remaining_quantity
        quantity = random.randint(1, remaining_quantity)
        order_items.append(item)
        order_quantities.append(quantity)
        remaining_quantity -= quantity

    return order_items, order_quantities
    
def single_order_publisher(table_id):
    # Inizializza il nodo publisher
    rospy.init_node(f'table_{table_id}', anonymous=False)

    menu_items = request_menu()
    if not menu_items:
        rospy.logwarn("Could not retrieve menu. Exiting.")
        return
    
    rospy.loginfo(f"Choosing items from menu...")
    rospy.sleep(1)

    # Step 2: Create a random order with a maximum total quantity
    items, quantities = create_random_order(menu_items, max_total_quantity=4)

    # Crea il publisher per il topic 'order_topic' e il tipo di messaggio 'Order'
    pub = rospy.Publisher('/order_topic', order_msg, queue_size=10)
    rospy.sleep(1)

    ordermsg = order_msg()
    ordermsg.table_id = table_id
    ordermsg.items = items
    ordermsg.quantities = quantities

    # Log per confermare l'invio
    rospy.loginfo(f"Publishing order for table {ordermsg.table_id}: {list(zip(items, quantities))}")
    
    # Pubblica il messaggio una sola volta
    pub.publish(ordermsg)

    # Attendi un breve periodo per garantire che il messaggio sia trasmesso
    rospy.sleep(1)
    # Keep the node alive
    rospy.spin()
    #rospy.loginfo("Ordine pubblicato e nodo terminato.")

if __name__ == '__main__':
    try:
        single_order_publisher(str(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
