#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 26 16:17:37 2025

@author: louis-pierrehuynh
"""

import numpy as np
import open3d as o3d


# ------------------------------------------------------------------
# Fonction principale : reconstruit un mesh à partir d'un nuage 3D
# ------------------------------------------------------------------
def reconstruct_mesh_from_point_cloud(
    points: np.ndarray,
    method: str = "bpa",          # 'bpa' = Ball Pivoting (recommandé), 'poisson' = Poisson Reconstruction
    voxel_size: float | None = None,
    poisson_depth: int = 9,
    density_quantile: float = 0.01,
    bpa_radius: float | None = None,
):
    """
    Reconstruit un maillage 3D à partir d'un nuage de points.
    """

    # --------------------------------------------
    # Sécurisation : conversion en numpy float64
    # --------------------------------------------
    if not isinstance(points, np.ndarray):
        points = np.array(points, dtype=np.float64)
    else:
        points = points.astype(np.float64)

    # --------------------------------------------
    # Création de l'objet PointCloud Open3D
    # --------------------------------------------
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # --------------------------------------------
    # Optionnel : downsampling pour alléger le nuage
    # --------------------------------------------
    if voxel_size is not None and voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)

    # --------------------------------------------
    # Calcul des distances entre chaque point et son plus proche voisin
    # Sert à obtenir l'échelle moyenne du nuage
    # --------------------------------------------
    distances = pcd.compute_nearest_neighbor_distance()

    # Si aucune distance (nuage trop petit) -> erreur
    if len(distances) == 0:
        raise ValueError("Nuage de points trop petit pour estimer des normales.")

    # Distance moyenne entre voisins
    avg_dist = float(np.mean(distances))

    # Rayon utilisé pour estimer les normales (voisinage)
    radius_normals = 2.5 * avg_dist

    # --------------------------------------------
    # Estimation des normales
    # --------------------------------------------
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_normals,   # sphère de recherche autour de chaque point
            max_nn=30                # max voisins utilisés
        )
    )

    # Orientation cohérente des normales (évite les normales inversées)
    pcd.orient_normals_consistent_tangent_plane(30)

    mesh = None  # mesh par défaut

    # ------------------------------------------------------------------
    #  Méthode 1 : Ball Pivoting Algorithm (stable, conseillé)
    # ------------------------------------------------------------------
    if method.lower() == "bpa":

        # Si aucun rayon fourni, on en déduit automatiquement un
        if bpa_radius is None:
            bpa_radius = 3.0 * avg_dist

        # Liste de rayons utilisés par BPA
        radii = o3d.utility.DoubleVector([
            bpa_radius,
            bpa_radius * 2.0
        ])

        # Reconstruction du mesh
        mesh_bpa = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, radii
        )

        # Sécurisation
        if mesh_bpa is None:
            raise RuntimeError("Ball Pivoting a échoué (mesh = None).")

        mesh = mesh_bpa

    # ------------------------------------------------------------------
    #  Méthode 2 : Poisson Reconstruction (plus lourde, moins stable)
    # ------------------------------------------------------------------
    elif method.lower() == "poisson":

        # Reconstruction poisson
        mesh_poisson, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=poisson_depth
        )

        if mesh_poisson is None:
            raise RuntimeError("Poisson Reconstruction a renvoyé None.")

        # Conversion densité en array numpy
        densities = np.asarray(densities)

        # On supprime les zones de faible densité (bruit)
        threshold = np.quantile(densities, density_quantile)
        mask = densities < threshold

        # Supprime les sommets bruités
        mesh = mesh_poisson.remove_vertices_by_mask(mask)

    else:
        raise ValueError("method doit être 'bpa' ou 'poisson'.")

    # ------------------------------------------------------------------
    # Nettoyage final du mesh (indispensable)
    # ------------------------------------------------------------------
    if mesh is None:
        raise RuntimeError("La reconstruction a retourné None.")

    mesh.remove_degenerate_triangles()      # triangles impossibles
    mesh.remove_duplicated_triangles()      # triangles en double
    mesh.remove_duplicated_vertices()       # sommets dupliqués
    mesh.remove_non_manifold_edges()        # edges problématiques
    mesh.compute_vertex_normals()           # recalcul des normales

    return mesh
# ---------------------------------------------------------
    # 1) Charger le nuage de points réel (déjà obtenu via LiDAR)
    #    → doit être un tableau NumPy de shape (N, 3)
    # ---------------------------------------------------------
    pts = mes_points  # Exemple : pts = np.load("lidar_points.npy")

    # Création du point cloud Open3D
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)

    print("Affichage du nuage de points...")
    o3d.visualization.draw_geometries([pcd])

    # ---------------------------------------------------------
    # 2) Reconstruction du maillage
    # ---------------------------------------------------------
    print("\n--- Reconstruction du maillage (BPA) ---")
    mesh = reconstruct_mesh_from_point_cloud(
        pts,
        method="bpa",     # méthode conseillée
        voxel_size=0.01   # down sampling optionnel
    )

    # ---------------------------------------------------------
    # 3) Visualisation du maillage reconstruit
    # ---------------------------------------------------------
    print("Affichage du maillage...")
    o3d.visualization.draw_geometries([mesh])

    # Sauvegarde éventuelle :
    # o3d.io.write_triangle_mesh("mesh_final.stl", mesh)