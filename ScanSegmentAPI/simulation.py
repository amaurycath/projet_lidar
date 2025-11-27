#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 25 15:00:21 2025

@author: louis-pierrehuynh
"""

import numpy as np
import open3d as o3d
import json
import numpy as np
import matplotlib.pyplot as plt
import nuage

points = nuage.données("acquisition/gobeletinv(h25;15,5,d).json")
points = nuage.nuage(points, D=155, plot=True)
plt.show()





# -------------------------------------------------------------
# 1. Fonction générique de reconstruction (version sécurisée)
# -------------------------------------------------------------
def reconstruct_mesh_from_point_cloud(
    points: np.ndarray,
    method: str = "bpa",
    voxel_size: float | None = None,
    poisson_depth: int = 9,
    density_quantile: float = 0.01,
    bpa_radius: float | None = None,
):
    """
    Reconstruit un maillage à partir d'un nuage de points dense.
    method = "bpa" par défaut car plus robuste.
    """
    
    # --- Assurer que points est un ndarray float64
    if not isinstance(points, np.ndarray):
        points = np.array(points, dtype=np.float64)
    else:
        points = points.astype(np.float64)

    # --- Créer le nuage
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # -------------------------------------------------------
    # Downsampling (optionnel)
    # -------------------------------------------------------
    if voxel_size is not None and voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)

    # -------------------------------------------------------
    # Normales
    # -------------------------------------------------------
    distances = pcd.compute_nearest_neighbor_distance()
    if len(distances) == 0:
        raise ValueError("Nuage de points trop petit pour estimer des normales.")

    avg_dist = float(np.mean(distances))
    radius_normals = 2.5 * avg_dist

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_normals,
            max_nn=30
        )
    )
    pcd.orient_normals_consistent_tangent_plane(30)

    mesh = None

    # -------------------------------------------------------
    #  Reconstruction - Méthode BPA (stable)
    # -------------------------------------------------------
    if method.lower() == "bpa":
        if bpa_radius is None:
            bpa_radius = 3.0 * avg_dist

        radii = o3d.utility.DoubleVector([bpa_radius, bpa_radius * 2.0])
        mesh_bpa = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, radii
        )

        if mesh_bpa is None:
            raise RuntimeError(
                "Ball Pivoting a échoué : mesh_bpa = None.\n"
                "Vérifie que ton nuage n'est pas vide."
            )

        mesh = mesh_bpa

    # -------------------------------------------------------
    #  Reconstruction - Méthode Poisson (moins stable)
    # -------------------------------------------------------
    elif method.lower() == "poisson":

        mesh_poisson, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=poisson_depth
        )

        if mesh_poisson is None:
            raise RuntimeError(
                "Poisson Reconstruction a renvoyé None.\n"
                "Essaye : voxel_size plus grand, poisson_depth plus petit, "
                "ou method='bpa'."
            )

        densities = np.asarray(densities)
        threshold = np.quantile(densities, density_quantile)
        mask = densities < threshold
        mesh = mesh_poisson.remove_vertices_by_mask(mask)

    else:
        raise ValueError("method doit être 'bpa' ou 'poisson'.")

    # -------------------------------------------------------
    # Nettoyage de sécurité
    # -------------------------------------------------------
    if mesh is None:
        raise RuntimeError("La reconstruction a retourné None.")

    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    mesh.compute_vertex_normals()

    return mesh


# -------------------------------------------------------------
# 2. Génération d’un nuage de points dense
# -------------------------------------------------------------
def generate_dense_random_point_cloud(shape="sphere", n_points=200_000, noise=0.002):
    """
    Génère un nuage dense pour tester la reconstruction.
    shape : "sphere" | "cylinder" | "blob"
    """
    
    if shape == "sphere":
        directions = np.random.normal(size=(n_points, 3))
        directions /= np.linalg.norm(directions, axis=1).reshape(-1, 1)
        pts = directions  # rayon = 1

    elif shape == "cylinder":
        theta = np.random.uniform(0, 2*np.pi, n_points)
        z = np.random.uniform(-1, 1, n_points)
        x = np.cos(theta)
        y = np.sin(theta)
        pts = np.vstack([x, y, z]).T

    elif shape == "blob":
        pts = np.random.normal(size=(n_points, 3))
        pts /= np.max(np.linalg.norm(pts, axis=1))

    else:
        raise ValueError("shape doit être sphere, cylinder ou blob")

    pts += np.random.normal(scale=noise, size=pts.shape)
    return pts


# -------------------------------------------------------------
# 3. Pipeline complet : affichage nuage + maillage
# -------------------------------------------------------------
if __name__ == "__main__":
    print("\n--- Génération du nuage de points ---")
    pts = points

    # Création du point cloud Open3D
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)

    print("Affichage du nuage de points…")
    o3d.visualization.draw_geometries([pcd])

    print("\n--- Reconstruction du maillage (BPA) ---")
    mesh = reconstruct_mesh_from_point_cloud(
        pts,
        method="bpa",     # stable
        voxel_size=0.01   # optionnel
    )

    print("Affichage du maillage…")
    o3d.visualization.draw_geometries([mesh])

    # Pour sauvegarder le maillage :
    # o3d.io.write_triangle_mesh("mesh_test.stl", mesh)
    