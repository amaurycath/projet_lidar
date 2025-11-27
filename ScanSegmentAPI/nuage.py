#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 24 11:49:01 2025

@author: louis-pierrehuynh
"""

import json
import numpy as np
import matplotlib.pyplot as plt


def données(fichier):
    # Charger le fichier JSON
    with open(fichier, "r") as f:
        data = json.load(f)
    
    # Extraction des colonnes
    theta = np.array([d["angle"] for d in data])      # en degrés
    R = np.array([d["distance"] for d in data])    # en centimètres
    phi = np.array([d["angle_moteur"] for d in data]) # en degrés
    
    points = np.vstack((R, theta,phi)).T # on transpose pour avoir le tableau dans le bon sens 
  
    return points
  


def nuage(L, D=155, plot=True): # L est le tableau de données 3D [R, Thétha, Phi]
                                # True si on veut afficher le plot 
    
    L = np.array(L)             # conversion en array numpy 
    R = L[:, 0]
    theta = np.deg2rad(L[:, 1]) # passage des angles en radians
    phi = np.deg2rad(L[:, 2])
    
    X = (D - R*np.cos(theta)) * np.sin(phi)     # application des conversions en coordonnées cartésiennes 
    Y = -(D - R*np.cos(theta)) * np.cos(phi)
    Z = R * np.sin(theta)
    
    cartesian_points = np.column_stack((X, Y, Z)) 
    
    """
    # Filtrage cubique
    mask = (Y >= -200) & (Y <= 200) & (X>=-200) &(X<=200) 
    X, Y, Z = X[mask], Y[mask], Z[mask]
    """
    if plot:
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
      
        
        # Amélioration de la lisibilité
        
        ax.scatter(X, Y, Z, s=3, c='dodgerblue', alpha=0.6, edgecolors='w', linewidths=0.2)
        
        ax.set_xlabel("X", fontsize=12)
        ax.set_ylabel("Y", fontsize=12)
        ax.set_zlabel("Z", fontsize=12)
        ax.set_title("Nuage de points 3D", fontsize=14)
        ax.grid(True)
        ax.view_init(elev=20, azim=45)  # angle de vue ajusté


        max_range = np.ptp([X, Y, Z], axis=1).max() / 2 
        mx, my, mz = np.mean(X), np.mean(Y), np.mean(Z) 
        ax.set_xlim(mx - max_range, mx + max_range) 
        ax.set_ylim(my - max_range, my + max_range) 
        ax.set_zlim(mz - max_range, mz + max_range) 
        ax.set_box_aspect([1, 1, 1]) 



        plt.tight_layout()
        plt.show
     
    return cartesian_points