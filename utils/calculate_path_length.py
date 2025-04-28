def calculate_path_length(path):
    if len(path) < 2:
        return 0.0
    total_length = 0.0
    for i in range(len(path) - 1):
        current_point = np.array(path[i])
        next_point = np.array(path[i + 1])
        distance = np.linalg.norm(next_point - current_point)
        total_length += distance
    return total_length
