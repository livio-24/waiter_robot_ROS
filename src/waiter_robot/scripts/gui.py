#!/usr/bin/env python

import csv
import tkinter as tk
from tkinter import messagebox
import rospy
from order_pub import publish_order
from waiter_robot.srv import request_bill, request_billResponse
from waiter_robot.msg import menu_item

def load_menu_from_csv(file_path):
    menu = []
    with open(file_path, 'r') as file:
        reader = csv.DictReader(file, delimiter=';')
        for row in reader:
            menu.append({"nome": row["Nome"], "prezzo": float(row["Prezzo"])})
    return menu



# Funzione per gestire la selezione del menu
def show_menu(table):
    # Finestra del menu per il tavolo selezionato
    menu_window = tk.Toplevel()
    menu_window.title(f"Menu")
    menu = load_menu_from_csv("/home/livio24/waiter_robot_ROS/src/waiter_robot/dataset/menu.csv")
    if not menu:
        rospy.logerr("menu error")

    # Area per visualizzare il menu
    tk.Label(menu_window, text=f"Menu", font=("Arial", 14, "bold")).pack(pady=10)
    
    # Frame per selezionare gli ordini
    menu_frame = tk.Frame(menu_window)
    menu_frame.pack(pady=10)
    
    # Creare checkbox per ogni voce del menu
    selected_items = {}
    for item in menu:
        print(item)
        var = tk.BooleanVar()  # Variabile per memorizzare lo stato della checkbox
        chk = tk.Checkbutton(menu_frame, text=f"{item['nome']} - {item['prezzo']}€", variable=var)
        chk.pack(anchor="w")
        selected_items[item["nome"]] = (var, item)


    # Bottone per confermare
    # Funzione per confermare l'ordine
    def confirm_order():
        current_order = []
        for name, (var, item) in selected_items.items():
            if var.get():  # Se la checkbox è selezionata
                current_order.append(item)
        
        if current_order:
            # Pubblica l'ordine tramite il nodo ROS
            publish_order(table, current_order)
            menu_window.destroy()
            messagebox.showinfo("Ordine Confermato", f"Ordine inviato per {table}:\n" +
                                "\n".join([f"{item['nome']} - {item['prezzo']}€" for item in current_order]))
            
    tk.Button(menu_window, text="Conferma Ordine", command=confirm_order).pack(pady=10)

    active_orders = rospy.get_param('/orders', {})
    if table in active_orders and len(active_orders[table]) > 0:
        tk.Button(menu_window, text="Request Bill", command=lambda t = table: calculate_bill(t)).pack(pady=10)

# Client bill service
def calculate_bill(table):
    try:
        rospy.wait_for_service('/request_bill')  # Aspetta che il servizio sia disponibile
        requestBill = rospy.ServiceProxy('/request_bill', request_bill)  # Crea il client del servizio
        
        # Esegui la chiamata al servizio
        response = requestBill(table_id=table)
        
        # Mostra il risultato
        if response.success:
            total_msg = f"Conto per il tavolo {table}:\n"
            total_msg += f"{response.items}:\n"
            total_msg += f"Totale: €{response.total:.2f}"
            messagebox.showinfo("Conto", total_msg)
        else:
            messagebox.showerror("Errore", response.message)
    except rospy.ServiceException as e:
        messagebox.showerror("Errore", f"Servizio non disponibile: {e}")



# Creare la finestra principale
root = tk.Tk()
root.title("Robot waiter")

# Creare un pulsante per ogni tavolo
for table_id in range(1,9):
    btn = tk.Button(root, text=f'table_{table_id}', font=("Arial", 12), width=20, 
                    command=lambda t=f'table_{table_id}': show_menu(t))
    btn.pack(pady=10)

rospy.init_node('gui_node', anonymous=True)
# Avviare la GUI
root.mainloop()


