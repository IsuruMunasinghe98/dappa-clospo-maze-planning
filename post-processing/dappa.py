def simplify_robot_path(path, grid):
    if len(path) < 3:  # If the path is too short, no simplification needed
        return path

    simplified_path = [path[0]]  # Always start with the first point

    for i in range(1, len(path) - 1):  # Process points from 2nd to 2nd-to-last
        prev_point = path[i - 1]
        current_point = path[i]
        next_point = path[i + 1]

        # Calculate vectors from previous to current and current to next points
        vector1 = ((current_point[0] - prev_point[0]), current_point[1] - prev_point[1])
        vector2 = ((next_point[0] - current_point[0]), next_point[1] - current_point[1])

        # Calculate dot product and cross product for angle and rotation direction
        dot_product = vector1[0] * vector2[0] + vector1[1] * vector2[1]
        cross_product_2d = vector1[0] * vector2[1] - vector1[1] * vector2[0] # Determining rotation direction in a 2D space

        # Calculate vector magnitudes
        magnitude1 = (vector1[0] ** 2 + vector1[1] ** 2) ** 0.5
        magnitude2 = (vector2[0] ** 2 + vector2[1] ** 2) ** 0.5

        # Calculate angle between vectors
        if magnitude1 == 0 or magnitude2 == 0:
            cosine_angle = -2  # Handle potential division by zero
        else:
            cosine_angle = dot_product / (magnitude1 * magnitude2)

        import math  # Import math inside the loop.
        angle_degrees = round(math.degrees(math.acos(cosine_angle)), 1)

        # Determine rotation direction (left or right)
        rotate_side = "NONE"
        if cross_product_2d > 0:
            rotate_side = "LEFT"
        elif cross_product_2d < 0:
            rotate_side = "RIGHT"

        # Check if the angle is small and if the robot can skip the current point
        if 0 < angle_degrees <= 90:
            if prev_point[1] == current_point[1] or current_point[1] == next_point[1]:
                check_right = (prev_point[0] > next_point[0] and rotate_side == "RIGHT") or (prev_point[0] <= next_point[0] and rotate_side == "LEFT")
                check_left = (prev_point[0] > next_point[0] and rotate_side == "LEFT") or (prev_point[0] <= next_point[0] and rotate_side == "RIGHT")

                if check_right and current_point[1] + 1 < len(grid[0]) and grid[current_point[0]][current_point[1] + 1] == 1:
                    simplified_path.append([current_point[0], current_point[1] + K * 0.5])
                    continue
                elif check_left and current_point[1] - 1 >= 0 and grid[current_point[0]][current_point[1] - 1] == 1:
                    simplified_path.append([current_point[0], current_point[1] - K * 0.5])
                    continue
            elif prev_point[0] == current_point[0] or current_point[0] == next_point[0]:
                check_down = (prev_point[1] < next_point[1] and rotate_side == "RIGHT") or (prev_point[1] >= next_point[1] and rotate_side == "LEFT")
                check_up = (prev_point[1] < next_point[1] and rotate_side == "LEFT") or (prev_point[1] >= next_point[1] and rotate_side == "RIGHT")

                if check_down and current_point[0] + 1 < len(grid) and grid[current_point[0] + 1][current_point[1]] == 1:
                    simplified_path.append([current_point[0] + K * 0.5, current_point[1]])
                    continue
                elif check_up and current_point[0] - 1 >= 0 and grid[current_point[0] - 1][current_point[1]] == 1:
                    simplified_path.append([current_point[0] - K * 0.5, current_point[1]])
                    continue
        simplified_path.append(current_point)

    simplified_path.append(path[-1])
    return simplified_path
