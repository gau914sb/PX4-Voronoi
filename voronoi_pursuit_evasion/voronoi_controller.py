"""
Voronoi Pursuit-Evasion Controllers
Adapted from MultiAgentPE repository for PX4 integration
"""

import numpy as np


def voronoi_corner_new(A, dimension=3):
    """
    Compute Voronoi cell vertex from pursuer relative positions
    
    Args:
        A: Array of relative positions forming this corner
        dimension: 2 or 3 for 2D/3D
        
    Returns:
        Voronoi corner position (point equidistant from pursuers)
    """
    if dimension == 3:
        p1, p2, p3 = A
        b = np.array([[p1 @ p1], [p2 @ p2], [p3 @ p3]])
        x = np.linalg.solve(A, 0.5*b).flatten()
        return x
    elif dimension == 2:
        p1, p2 = A
        b = np.array([[p1 @ p1], [p2 @ p2]])
        x = np.linalg.solve(A, 0.5*b).flatten()
        return x


def pursuer_voronoi_controller(relative_poses, strategy=1, dimension=3, gamma=2, norm_bound=1.0):
    """
    Voronoi-based pursuit controller
    
    Args:
        relative_poses: Array of pursuer positions relative to evader (n_pursuers x dimension)
        strategy: Control strategy (1 or 2)
        dimension: 2 or 3 for 2D/3D
        gamma: Control law parameter
        norm_bound: Maximum velocity magnitude
        
    Returns:
        pursuer_velocities: Velocity commands for each pursuer
        voronoi_corners: Computed Voronoi cell vertices
        A: Relative position matrices
        c: Barycentric coordinates
        furthest_corner_idx: Index of furthest corner (-1 for strategy 2)
        K: Control gain vectors
    """
    n_pursuers = dimension + 1  # 4 for 3D, 3 for 2D
    voronoi_corners = np.zeros((n_pursuers, dimension))
    A = np.zeros((n_pursuers, dimension, dimension))
    c = np.zeros((n_pursuers, dimension))
    
    # Compute Voronoi corners and barycentric coordinates
    for i in range(n_pursuers):
        A[i] = np.array(([relative_poses[(i + j) % n_pursuers] for j in range(dimension)]))
        voronoi_corners[i] = voronoi_corner_new(A[i], dimension=dimension)
        c[i] = np.linalg.solve(A[i].T, voronoi_corners[i])

    furthest_corner_idx = -1
    pursuer_velocities = np.zeros((n_pursuers, dimension))
    K = np.zeros((n_pursuers, dimension))
    
    # Compute control gains
    for i in range(dimension + 1):
        for j in range(dimension):
            K[(i+j) % n_pursuers] += c[i][j] * (np.linalg.norm(voronoi_corners[i]))**(gamma - 2) * \
                                      (relative_poses[(i+j) % n_pursuers] - voronoi_corners[i])
    
    # Strategy 1 - Three chase the furthest voronoi corner, one chases the evader
    if strategy == 1:
        voronoi_distances = np.linalg.norm(voronoi_corners, axis=1)
        sorted_order = np.argsort(voronoi_distances)
        furthest_corner_idx = sorted_order[-1]

        # Pursuers forming furthest corner move toward it
        for i in range(furthest_corner_idx, furthest_corner_idx + dimension):
            pursuer_velocities[i % n_pursuers, :] = (voronoi_corners[furthest_corner_idx] - 
                                                       relative_poses[i % n_pursuers, :])

        # Remaining pursuer chases evader directly
        pursuer_velocities[(furthest_corner_idx + dimension) % n_pursuers, :] = \
            -relative_poses[(furthest_corner_idx + dimension) % n_pursuers, :]
        
        # Normalize velocities
        for i in range(n_pursuers):
            pursuer_velocities[i] /= np.linalg.norm(pursuer_velocities[i])
    
    # Strategy 2 - Proposed in the Paper (normalized K-based control)
    elif strategy == 2:
        for i in range(dimension + 1):
            K_norm = np.linalg.norm(K[i])
            if K_norm > 1e-6:  # Avoid division by zero
                pursuer_velocities[i] = -(K[i] / K_norm) * norm_bound
            else:
                # Fallback: move toward evader
                pursuer_velocities[i] = -relative_poses[i] / (np.linalg.norm(relative_poses[i]) + 1e-6) * norm_bound
    else:
        raise ValueError(f'Pursuer Strategy {strategy} does not exist')
    
    return pursuer_velocities, voronoi_corners, A, c, furthest_corner_idx, K


def evader_voronoi_controller(voronoi_corners, K, strategy=1, norm_bound=1.0):
    """
    Voronoi-based evasion controller
    
    Args:
        voronoi_corners: Computed Voronoi cell vertices
        K: Control gain vectors from pursuer controller
        strategy: Evasion strategy (1 or 2)
        norm_bound: Maximum velocity magnitude
        
    Returns:
        evader_vel: Velocity command for evader
    """
    if strategy == 1:
        # Move toward furthest Voronoi corner (escape furthest)
        voronoi_distances = np.linalg.norm(voronoi_corners, axis=1)
        sorted_order = np.argsort(voronoi_distances)
        idx1 = sorted_order[-1]
        evader_vel = voronoi_corners[idx1, :].copy()
        vel_norm = np.linalg.norm(evader_vel)
        if vel_norm > 1e-6:
            evader_vel /= vel_norm
        else:
            evader_vel = np.zeros_like(evader_vel)
            
    elif strategy == 2:
        # Move opposite to sum of pursuer control gains
        evader_vel = np.sum(K, axis=0)
        vel_norm = np.linalg.norm(evader_vel)
        if vel_norm > 1e-6:
            evader_vel /= vel_norm
        else:
            evader_vel = np.zeros_like(evader_vel)
            
    elif strategy == 3:
        raise NotImplementedError("Strategy 3 not implemented yet")
    else:
        raise ValueError(f'Evader strategy {strategy} does not exist')
    
    evader_vel *= norm_bound
    return evader_vel
