"""
Main Script: Path Planning, Simplification, Smoothing, and Visualization

Before running this file:
---------------------------------------------
1. Ensure all helper functions are loaded:
   - path_planning_algorithms/: path planning functions
   - path_smoothing/: contains smoothing functions like P-controller, Bezier curve generators
   - post_processing/: contains path simplification and optimization methods
   - utils/: contains utility functions such as distance calculators

2. Make sure to add your grid environments and source-destination points
   into the file: grid_data.py

This main script then:
---------------------------------------------
- Finds paths using origibal algorithm
- Simplifies the paths
- Applies smoothing techniques (P-controller, Quadratic Bezier, Rational Quadratic Bezier)
- Calculates path lengths
- Plots all results for easy visualization and comparison
"""

import matplotlib.pyplot as plt
import numpy as np
from math import pi
import matplotlib.pyplot as plt
import numpy as np
import math
import heapq

num_grids = len(grid_data)
fig, axes = plt.subplots(1, num_grids, figsize=(5 * num_grids, 5))

# If there's only one grid, wrap axes in a list for easy iteration
if num_grids == 1:
    axes = [axes]

# Loop through each grid and perform path planning and visualization
for i, (ax, grid) in enumerate(zip(axes, grid_data)):
    simplified_paths = []
    array = np.array(grid)

    # Find Dijkstra Path
    dijkstra = find_dijkstra(array, src[i], dest[i])

    # Simplify using custom robot-aware simplification
    simplified_dijkstra_1 = simplify_robot_path(dijkstra, grid)
    
    # Further simplify using line-of-sight based method
    simplified_dijkstra = simplify_path_1(grid, simplified_dijkstra_1)
    
    # Apply P-Controller for smoother intermediate points
    controlled_simplified_dijkstra = apply_p_controller(simplified_dijkstra, grid)
    
    # Apply Quadratic and Rational Quadratic Bezier Smoothing
    expanded_intermediate_quadratic_bezier_curve_points, expanded_intermediate_rational_quadratic_bezier_curve_points = get_intermediate_quadratic_bezier_curve_points(simplified_dijkstra)

    # Calculate the lengths of different paths
    path_lengths = calculate_path_length(dijkstra)
    path_lengths_simplified = calculate_path_length(simplified_dijkstra)
    path_lengths_controlled = calculate_path_length(controlled_simplified_dijkstra)
    path_lengths_quadratic_bezier = calculate_path_length(expanded_intermediate_quadratic_bezier_curve_points)
    path_lengths_rational_quadratic_bezier = calculate_path_length(expanded_intermediate_rational_quadratic_bezier_curve_points)

    # Plotting the grid
    ax.imshow(array, cmap="gray", origin="upper")

    # Extract X, Y coordinates
    x_dijkstra = [wp[1] for wp in dijkstra]
    y_dijkstra = [wp[0] for wp in dijkstra]

    x_simplified = [wp[1] for wp in simplified_dijkstra]
    y_simplified = [wp[0] for wp in simplified_dijkstra]

    x_controlled = [wp[1] for wp in controlled_simplified_dijkstra]
    y_controlled = [wp[0] for wp in controlled_simplified_dijkstra]

    x_quadratic_bezier = [wp[1] for wp in expanded_intermediate_quadratic_bezier_curve_points]
    y_quadratic_bezier = [wp[0] for wp in expanded_intermediate_quadratic_bezier_curve_points]

    x_rational_quadratic_bezier = [wp[1] for wp in expanded_intermediate_rational_quadratic_bezier_curve_points]
    y_rational_quadratic_bezier = [wp[0] for wp in expanded_intermediate_rational_quadratic_bezier_curve_points]

    # Plot different paths
    ax.plot(x_dijkstra, y_dijkstra, 'r-', label='Dijkstra')
    ax.plot(x_simplified, y_simplified, 'g-', label='Simplified Dijkstra')
    ax.plot(x_controlled, y_controlled, 'b-', label='P-Controlled Simplified Dijkstra') # P-controller
    ax.plot(x_quadratic_bezier, y_quadratic_bezier, 'm-', label='Quadratic Bezier Curve') # Quadratic Bezier Curve
    ax.plot(x_rational_quadratic_bezier, y_rational_quadratic_bezier, 'c-', label='Rational Quadratic Bezier Curve') # Rational Quadratic Bezier Curve

    ax.legend()
    ax.set_title(f"Grid {i+1} Path Planning Visualization")
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

plt.tight_layout()
plt.show()
