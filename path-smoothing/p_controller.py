def apply_p_controller(path, grid=None):
    if len(path) < 2:
        return path

    controlled_path = []
    current_position = np.array(path[0])
    path_index = 1
    controlled_path.append(current_position.tolist())

    steps = 0
    while path_index < len(path) and steps < STEP_LIMIT:
        target_position = np.array(path[path_index])
        error = target_position - current_position
        distance = np.linalg.norm(error)

        if distance < THRESHOLD:
            path_index += 1
            continue

        velocity = KP * error
        max_step = 0.2
        if np.linalg.norm(velocity) > max_step:
            velocity = (velocity / np.linalg.norm(velocity)) * max_step

        current_position = current_position + velocity
        controlled_path.append(current_position.tolist())
        steps += 1
    return controlled_path
