import numpy as np
# import open3d as o3d


def getRoundedThresholdv1(a, Minclip):  
    return np.round(a / Minclip) * Minclip


def get_rsr_signature(lidar_points_orig, DRONE_CENTER, NUM_PTS=5, GRANULARITY=2, REACH_RANGE=6, RADIUS=1, plot_lidar=False):
    indices = np.arange(0, NUM_PTS, dtype=float) + 0.5
    phi = np.arccos(1 - 2*indices/NUM_PTS)
    theta = np.pi * (1 + 5**0.5) * indices
    x, y, z = np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)

    beam_dvs = np.column_stack((x, y, z)) # direction vector list
    
    lidar_points = lidar_points_orig + DRONE_CENTER
    lidar_boundary_points = (beam_dvs * REACH_RANGE)

    STEPS = REACH_RANGE // GRANULARITY # 30 steps, of 2 granularity each all the way till 60
    
    truncated_lidar_points = np.zeros(lidar_boundary_points.shape)
    for i, orig_point in enumerate(lidar_boundary_points):
        for t in range(STEPS, 0, -1): # 5, 4, 3, 2, 1
            t_point = orig_point / t
            
            inside = (lidar_points-t_point)**2
            distances = np.sum(inside, axis=1)
            points_in_radius = np.sum(distances<RADIUS**2)
            if points_in_radius > 0:
                truncated_lidar_points[i] = t_point
                break
            else:
                truncated_lidar_points[i] = orig_point

    rsr_signature = np.linalg.norm(truncated_lidar_points, axis=1)
    rsr_signature = getRoundedThresholdv1(rsr_signature, GRANULARITY)
    
    # if plot_lidar:
    #     d = o3d.geometry.PointCloud()
    #     d.points = o3d.utility.Vector3dVector(beam_dvs)   
    #     d.paint_uniform_color([1, 0, 0]) 

    #     l = o3d.geometry.PointCloud()
    #     l.points = o3d.utility.Vector3dVector(lidar_points)
    #     l.paint_uniform_color([0.67, 1, 0]) 

    #     tl = o3d.geometry.PointCloud()
    #     tl.points = o3d.utility.Vector3dVector(truncated_lidar_points)
    #     tl.paint_uniform_color([0, 0.3, 0.206]) 

    #     o3d.visualization.draw_geometries([d, l, tl])

    return rsr_signature