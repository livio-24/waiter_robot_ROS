#!/usr/bin/env python
import threading
import csv
import tkinter as tk
from tkinter import ttk, messagebox
import rospy
#from order_pub import publish_order
from waiter_robot.srv import request_bill, request_billResponse
from waiter_robot.msg import order_msg, menu_item

from ttkthemes import ThemedTk


#active_orders = rospy.get_param('/orders', {})

# Funzione per caricare il menu
def load_menu_from_csv(file_path):
    menu = []
    with open(file_path, 'r') as file:
        reader = csv.DictReader(file, delimiter=';')
        for row in reader:
            menu.append({"name": row["Nome"], "price": float(row["Prezzo"])})
    return menu

# menu selection
def show_menu(table):
    menu_window = tk.Toplevel()
    menu_window.title(f"Menu for {table}")
    menu = load_menu_from_csv("/home/livio24/waiter_robot_ROS/src/waiter_robot/dataset/menu.csv")
    
    if not menu:
        rospy.logerr("Menu error")
        messagebox.showerror("Error", "Menu not found!")
        return

    ttk.Label(menu_window, text=f"Menu for {table}", font=("Arial", 14, "bold")).pack(pady=10)

    menu_frame = ttk.Frame(menu_window)
    menu_frame.pack(pady=10)
    
    # create checkbox for each menu item
    selected_items = {}
    for item in menu:
        var = tk.BooleanVar()
        chk = ttk.Checkbutton(menu_frame, text=f"{item['name']} - {item['price']}€", variable=var)
        chk.pack(anchor="w")
        selected_items[item["name"]] = (var, item)


    def confirm_order():
        current_order = []
        for name, (var, item) in selected_items.items():
            if var.get():
                current_order.append(item)
        
        if current_order:
            publish_order(table, current_order)
            menu_window.destroy()
            messagebox.showinfo("Order Confirmed", f"Order sent for {table}:\n" +
                                "\n".join([f"{item['name']} - {item['price']}€" for item in current_order]))
            
    ttk.Button(menu_window, text="Confirm Order", command=confirm_order).pack(pady=10)



    # Button to request bill
    active_orders = rospy.get_param('/orders', {})
    if table in active_orders and len(active_orders[table]) > 0:
        ttk.Button(menu_window, text="Request Bill", command=lambda t=table: calculate_bill(t)).pack(pady=10)

# service client
def calculate_bill(table):
    try:
        rospy.wait_for_service('/request_bill')
        requestBill = rospy.ServiceProxy('/request_bill', request_bill)
        response = requestBill(table_id=table)
        
        if response.success:
            total_msg = f"bill for table {table}:\n"
            total_msg += f"{response.items}:\n"
            total_msg += f"Total: €{response.total:.2f}"
            messagebox.showinfo("Bill", total_msg)
        else:
            messagebox.showerror("Error", response.message)
    except rospy.ServiceException as e:
        messagebox.showerror("Error", f"Service not available: {e}")

def show_active_orders_window():
    # Finestra per visualizzare gli ordini attivi
    orders_window = tk.Toplevel()
    orders_window.title("Active Orders")
    orders_window.geometry("400x300")

    # Frame principale della finestra
    orders_frame = ttk.Frame(orders_window, padding=10)
    orders_frame.pack(expand=True, fill="both")

    # Dizionario per tenere traccia dei frame per ciascun tavolo
    table_labels = {}

    # Stile per i widget della finestra
    style = ttk.Style()
    style.configure("Card.TFrame", background="#f5f5f5", borderwidth=1, relief="solid")
    style.configure("TLabel", font=("Arial", 10))

    # Salvo lo stato precedente degli ordini (inizialmente vuoto)
    previous_orders = {}

    # Funzione per aggiornare gli ordini attivi
    def update_orders():
        nonlocal previous_orders  # Riferimento alla variabile esterna

        # Ottenere gli ordini attivi dal ROS Parameter Server
        try:
            orders = rospy.get_param('/orders', {})
        except KeyError:
            orders = {}

        # Confronta con lo stato precedente
        if orders != previous_orders:
            # Aggiungere o aggiornare i frame per ciascun tavolo
            for table_id, order_items in orders.items():
                # Se il tavolo non è ancora visibile, creiamo il frame
                if table_id not in table_labels:
                    card_frame = ttk.Frame(orders_frame, style="Card.TFrame", padding=10)
                    card_frame.pack(fill="x", pady=5, padx=5)

                    # Titolo del tavolo
                    title = ttk.Label(card_frame, text=f"Table: {table_id}", font=("Arial", 12, "bold"))
                    title.pack(anchor="w")

                    # Salviamo il frame creato per il tavolo
                    table_labels[table_id] = card_frame

                # Ora aggiorniamo gli ordini del tavolo esistente
                card_frame = table_labels[table_id]

                # Rimuoviamo tutti i widget di ordine precedenti
                for widget in card_frame.winfo_children()[1:]:  # Saltando il titolo
                    widget.destroy()

                # Aggiungiamo i nuovi ordini
                for item in order_items:
                    item_label = ttk.Label(card_frame, text=f"- {item}", font=("Arial", 10))
                    item_label.pack(anchor="w")

            # Rimuovere i frame dei tavoli che non hanno più ordini
            for table_id in list(table_labels.keys()):
                if table_id not in orders:
                    table_labels[table_id].destroy()
                    del table_labels[table_id]

            # Aggiornamento dello stato precedente
            previous_orders = orders  # Salva lo stato corrente per il prossimo ciclo

        # Pianificare il prossimo aggiornamento
        orders_frame.after(1000, update_orders)  # Aggiorna ogni 1 secondo

    # Avviare il primo aggiornamento
    update_orders()

def publish_order(table_id, menu_items):

    # Create pub
    pub = rospy.Publisher('/order_topic', order_msg, queue_size=10)
    rospy.sleep(1)

    ordermsg = order_msg()
    ordermsg.table_id = table_id
    ordermsg.items = [menu_item(name=item['name'], price=item['price']) for item in menu_items]

    rospy.loginfo(f"Publishing order for {ordermsg.table_id}: {menu_items}")
    
    pub.publish(ordermsg)

def init_gui():
    rospy.init_node('gui_node', anonymous=True)
    # Finestra principale
    root = ThemedTk(theme="breeze")
    root.title("Robot Waiter")

    # Stile per i pulsanti
    style = ttk.Style()
    style.configure("TButton", font=("Arial", 12), padding=5)

    # Layout a griglia per i pulsanti dei tavoli
    frame = ttk.Frame(root, padding=20)
    frame.pack(expand=True, fill="both")

    table_buttons = {}

    for i in range(4):
        for j in range(2):
            table_id = f"table_{i * 2 + j + 1}"
            btn = ttk.Button(frame, text=f"{table_id}", width=20, command=lambda t=table_id: show_menu(t))
            btn.grid(row=i, column=j, padx=10, pady=10)
            table_buttons[table_id] = btn


    ttk.Button(root, text="Start delivery", command=lambda: publish_order('delivery', [])).pack(pady=10)
    ttk.Button(root, text="Show Active Orders", command=show_active_orders_window).pack(pady=10)


    root.mainloop()

if __name__ == '__main__':
    init_gui()