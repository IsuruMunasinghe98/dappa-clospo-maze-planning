def simplify_path_1(grid, path):
  
    # Remove points that have non-integer coordinates
    integer_path = [tuple(map(int, point)) for point in path if all(isinstance(coord, int) for coord in point)]
    if not integer_path:
        return []

    # Initialize the simplified path with the start point
    simplified_path = [integer_path[0]]
    n = len(integer_path)
    i = 0

    while i < n - 1:
        j = i + 2  # Try to skip at least one point ahead
        while j < n:
            p1 = simplified_path[-1]  # Last point in the current simplified path
            p2 = integer_path[j]       # Candidate point to jump to
            can_simplify = True

            if p1[0] == p2[0]:  # Check if they are horizontally aligned (same row)
                row = p1[0]
                start_col = min(p1[1], p2[1])
                end_col = max(p1[1], p2[1])

                # Check all columns between p1 and p2 for obstacles
                for col in range(start_col + 1, end_col):
                    if not (0 <= row < len(grid) and 0 <= col < len(grid[0]) and grid[row][col] == 1):
                        can_simplify = False
                        break

            elif p1[1] == p2[1]:  # Check if they are vertically aligned (same column)
                col = p1[1]
                start_row = min(p1[0], p2[0])
                end_row = max(p1[0], p2[0])

                # Check all rows between p1 and p2 for obstacles
                for row in range(start_row + 1, end_row):
                    if not (0 <= row < len(grid) and 0 <= col < len(grid[0]) and grid[row][col] == 1):
                        can_simplify = False
                        break

            else:
                # Not aligned horizontally or vertically
                can_simplify = False

            if can_simplify:
                # If direct path is clear, move further ahead
                j += 1
            else:
                # If not, accept the last valid point before obstacle appeared
                simplified_path.append(integer_path[j - 1])
                i = j - 1
                break
        else:
            # If loop ended normally, append the final point
            simplified_path.append(integer_path[-1])
            break

    return simplified_path
