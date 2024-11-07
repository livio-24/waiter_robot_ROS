#!/usr/bin/env python

import rospy
from waiter_robot.msg import order_msg
from navigation import send_goal

# Variabili globali
ordini_in_attesa = []
ordini_in_coda = []  # Nuova lista per accodare ordini durante la consegna
raccolta_in_corso = False
in_consegna = False
timer_raccolta = None
RACCOLTA_ORDINI_TEMPO = 30  # Durata fase di raccolta ordini

def ordine_callback(msg):
    global raccolta_in_corso, in_consegna, ordini_in_attesa, ordini_in_coda

    # Crea un nuovo ordine
    ordine = {"table_id": msg.table_id, 
              "items": msg.items,
              "quantities": msg.quantities}

    if in_consegna:
        # Se il robot è già in consegna, accoda il nuovo ordine
        ordini_in_coda.append(ordine)
        rospy.loginfo(f"Ordine ricevuto durante la consegna. Accodato per il tavolo {ordine['table_id']}: {list(zip(ordine['items'], ordine['quantities']))}")
    else:
        # Se il robot è in fase di raccolta, aggiungi l'ordine alla lista principale
        ordini_in_attesa.append(ordine)
        rospy.loginfo(f"Ricevuto ordine per il tavolo {ordine['table_id']}: {list(zip(ordine['items'], ordine['quantities']))}")

        # Avvia la fase di raccolta se non è già in corso
        if not raccolta_in_corso:
            avvia_fase_raccolta()

def avvia_fase_raccolta():
    global raccolta_in_corso, timer_raccolta
    raccolta_in_corso = True
    rospy.loginfo("Iniziata fase di raccolta ordini per 30 secondi.")
    timer_raccolta = rospy.Timer(rospy.Duration(RACCOLTA_ORDINI_TEMPO), consegna, oneshot=True)

def consegna(event):
    global raccolta_in_corso, in_consegna, ordini_in_attesa, ordini_in_coda

    # Termina la fase di raccolta e passa alla fase di consegna
    raccolta_in_corso = False
    in_consegna = True
    rospy.loginfo("Fase di raccolta terminata. Inizio della consegna degli ordini.")

    # Unisci gli ordini in attesa con quelli accodati durante la consegna
    ordini_in_attesa.extend(ordini_in_coda)
    ordini_in_coda.clear()  # Svuota la coda temporanea

    # Naviga al bancone per raccogliere gli ordini
    rospy.loginfo("Navigazione al bancone.")
    send_goal('bancone')  # Naviga al bancone

    # Cicla su ogni ordine in attesa e invia il robot al tavolo appropriato
    while ordini_in_attesa:
        ordine = ordini_in_attesa.pop(0)  # Estrai il primo ordine dalla lista
        #table_id = ordine["table_id"]
        #items =  ordine['items']
        #qnt =  ordine['quantities']

        # Naviga al tavolo per consegnare l'ordine
        rospy.loginfo(f"Navigo verso il tavolo {ordine['table_id']} per consegnare: {list(zip(ordine['items'], ordine['quantities']))}")
        send_goal(ordine["table_id"])
        rospy.loginfo(f"Ordine consegnato al tavolo {ordine['table_id']}.")

    # Dopo aver consegnato tutti gli ordini, torna allo stato di attesa
    in_consegna = False
    rospy.loginfo("Tutti gli ordini sono stati consegnati.")
    rospy.sleep(1)
    # Se ci sono ordini in coda, riavvia una nuova fase di raccolta
    if ordini_in_coda:
        rospy.loginfo("Ci sono ordini accodati. Avvio una nuova fase di raccolta.")
        avvia_fase_raccolta()

def ordine_subscriber():
    # Inizializza il nodo
    rospy.init_node('ordine_subscriber', anonymous=True)

    # Crea il subscriber per il topic degli ordini
    rospy.Subscriber('order_topic', order_msg, ordine_callback)

    # Mantiene il nodo in esecuzione
    rospy.spin()

if __name__ == '__main__':
    try:
        ordine_subscriber()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f"Eccezione ROS: {e}")
