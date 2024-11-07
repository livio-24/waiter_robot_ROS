#!/usr/bin/env python
import rospy
from waiter_robot.srv import menu, menuResponse

# Define the menu options
MENU_ITEMS = ["coffee", "beer", "soda", "tea"]

def handle_get_menu_request(req):
    # Prepare the response with menu items and their prices
    response = menuResponse()
    response.item_names = MENU_ITEMS
    rospy.loginfo("Servizio Menu richiesto, inviando il menu.")
    return response

def get_menu_server():
    rospy.init_node('menu_server')
    # Define the service and bind it to the handle function
    service = rospy.Service('get_menu', menu, handle_get_menu_request)
    rospy.loginfo("Servizio Menu disponibile. In attesa di richieste.")
    rospy.spin()

if __name__ == '__main__':
    try:
        get_menu_server()
    except rospy.ROSInterruptException:
        pass
