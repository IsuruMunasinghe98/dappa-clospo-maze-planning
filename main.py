num_grids = len(grid_data)
fig, axes = plt.subplots(1, num_grids, figsize=(5 * num_grids, 5))

if num_grids == 1:
    axes = [axes]

for i, (ax, grid) in enumerate(zip(axes, grid_data)):
    simplified_paths = []
    array = np.array(grid)
    dijkstra = find_dijkstra(array, src[i], dest[i])
    simplified_dijkstra_1 = simplify_robot_path(dijkstra, grid)
    simplified_dijkstra = simplify_path_1(grid, simplified_dijkstra_1)
    controlled_simplified_dijkstra = apply_p_controller(simplified_dijkstra, grid)
    expanded_intermediate_quadratic_bezier_curve_points, expanded_intermediate_rational_quadratic_bezier_curve_points = get_intermediate_quadratic_bezier_curve_points(simplified_dijkstra)

    path_lengths = calculate_path_length(dijkstra)
    path_lengths_simplified = calculate_path_length(simplified_dijkstra)
    path_lengths_controlled = calculate_path_length(controlled_simplified_dijkstra)
    path_lengths_quadratic_bezier = calculate_path_length(expanded_intermediate_quadratic_bezier_curve_points)
    path_lengths_rational_quadratic_bezier = calculate_path_length(expanded_intermediate_rational_quadratic_bezier_curve_points)

    ax.imshow(array, cmap="gray", origin="upper")

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

    ax.plot(x_dijkstra, y_dijkstra, 'r-', label='Dijkstra')
    ax.plot(x_simplified, y_simplified, 'b-', label='Simplified Dijkstra')
    ax.plot(x_controlled, y_controlled, 'b-', label='P Controlled Simplified Dijkstra')
    ax.plot(x_quadratic_bezier, y_quadratic_bezier, 'b-', label='Quadratic Bezier Curve - Simplified Dijkstra')
    ax.plot(x_rational_quadratic_bezier, y_rational_quadratic_bezier, 'b-', label='Rational Quadratic Bezier Curve - Simplified Dijkstra')

    ax.plot(src[i][1], src[i][0], 'ro', markersize=8, label='Source')
    ax.plot(dest[i][1], dest[i][0], 'bo', markersize=8, label='Destination')

    ax.set_title(f"Path {i + 1}")
    ax.set_xticks([])
    ax.set_yticks([])

    ax.set_xlabel(f"Dijkstra: {path_lengths:.2f}\n Post-processed Simplified Path: {path_lengths_simplified:.2f}\n P-Controlled: {path_lengths_controlled:.2f}\n Quadratic Bezier Curve: {path_lengths_quadratic_bezier:.2f} \n Rational Quadratic Bezier Curve:  {path_lengths_rational_quadratic_bezier:.2f}", fontsize=10)

plt.tight_layout()
plt.show()
