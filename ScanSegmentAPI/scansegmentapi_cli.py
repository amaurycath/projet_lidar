# Copyright (c) 2023-2024 SICK AG
# SPDX-License-Identifier: MIT

# Port used for data streaming. Enter the port configured in your device.
PORT = 2115

# If UDP is configured this should be the IP of the receiver.
# If TCP is configured this should be the IP of the SICK device.
IP = "192.168.0.100"

# Select with which transport protocol the data should be received. Select "TCP" or "UDP".
TRANSPORT_PROTOCOL = "UDP"

import numpy as np
import scansegmentapi.msgpack as MSGPACKApi
from scansegmentapi.tcp_handler import TCPHandler
from scansegmentapi.msgpack_stream_extractor import MsgpackStreamExtractor
from scansegmentapi.udp_handler import UDPHandler
from collections import defaultdict
import time
import serial # Communication avec Arduino
import json

arduino = serial.Serial('COM4', 9600, timeout=1) # COM9 à modifier selon le port utilisé

# Fonction pour envoyer une commande à l'Arduino
def send_command(command):
    arduino.write(f"{command}\n".encode())
# send_command("start") pour envoyer un pas de moteur


def filtrer_lidar_par_seuil(acquisitions, seuil):
    filtered_all = []

    for tableau in acquisitions:      # boucle sur les 400 acquisitions
        filtered_tableau = []

        for angle, distance, anglemot in tableau:   # boucle sur les points
            if distance <= seuil and distance != 0:
                filtered_tableau.append([angle, distance, anglemot])

        filtered_all.append(filtered_tableau)

    return filtered_all

def mesurer_temps_execution(func, *args, **kwargs):
    start = time.perf_counter()
    result = func(*args, **kwargs)
    end = time.perf_counter()
    print(f"Temps d'exécution : {end - start:.6f} secondes")
    return result


def acquerir_frame(nb_segments=30):
    """Récupère la première frame complète et retourne un tableau [(angle,distance), ...]"""
    if TRANSPORT_PROTOCOL == "UDP":
        transportLayer = UDPHandler(IP, PORT, 65535)
    else:
        streamExtractor = MsgpackStreamExtractor()
        transportLayer = TCPHandler(streamExtractor, IP, PORT, 1024)

    receiver = MSGPACKApi.Receiver(transportLayer)

    segments, frameNumbers, segmentCounters = mesurer_temps_execution(
        receiver.receive_segments, nb_segments
    )

    receiver.close_connection()

    print(f"Nombre total de segments reçus : {len(segments)}")

    tableau = []
    frames = defaultdict(list)
    for segment in segments:
        frames[segment["FrameNumber"]].append(segment)

    found_complete = False
    for frameNumber in sorted(frames.keys()):
        frameSegments = frames[frameNumber]
        if len(frameSegments) == 10:
            print(f"Frame complète trouvée : {frameNumber} avec {len(frameSegments)} segments")
            for segment in frameSegments:
                segmentCounter = segment["SegmentCounter"]
                for scan in segment["SegmentData"]:
                    channelTheta = scan["ChannelTheta"]
                    distances = scan["Distance"][0]
                    tableau.extend(
                        (float(np.rad2deg(angle)), float(distance))
                        for angle, distance in zip(channelTheta, distances)
                    )
            found_complete = True
            break
        else:
            print(f"Frame incomplète : {frameNumber} ({len(frameSegments)} segments)")

    if not found_complete:
        print("Aucune frame complète reçue dans ce lot.")

    print(f"\nLongueur totale de tableau = {len(tableau)}")
    return tableau


if __name__ == "__main__":
    intervalle = 0.5  # secondes entre deux acquisitions

    # Grand tableau contenant les 400 acquisitions
    acquisitions = []   # => liste de 400 sous-listes

    try:
        for k in range(0, 400):   # 0 à 399 inclus
            tableau = acquerir_frame(nb_segments=30)  # Renvoie une liste de tuples (angle, distance)

            angle_rotation_moteur = 0.9 * k

            # Ajout de la 3e valeur dans chaque point
            tableau_modifie = [
                (angle, distance, angle_rotation_moteur) 
                for angle, distance in tableau
            ]

            # On ajoute le tableau modifié dans la liste principale
            acquisitions.append(tableau_modifie)

            print(f"Acquisition {k:03d} terminée : {len(tableau_modifie)} points reçus")

            send_command("start")  # Commande Arduino
            time.sleep(intervalle)

    except KeyboardInterrupt:
        print("Acquisition interrompue par l'utilisateur")

def generer_json_multi(acquisitions):
    all_data = []
    for tableau in acquisitions:
        for angle, distance, mot in tableau:
            all_data.append({
                "angle": angle,
                "distance": distance,
                "angle_moteur": mot
            })
    return json.dumps(all_data, indent=4)

acqui_filtre=filtrer_lidar_par_seuil(acquisitions,250)
json_resultat = generer_json_multi(acqui_filtre)
with open("lidar_data.json", "w") as f:
    f.write(json_resultat)
print(acqui_filtre)
arduino.close() #Ferme le port