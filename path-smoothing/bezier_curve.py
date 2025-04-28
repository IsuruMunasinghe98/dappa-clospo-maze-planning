def bezier_curve(P0, P1, P2, num_points=10):
    curve = []
    for t in np.linspace(0, 1, num_points):
        point = (1 - t)**2 * np.array(P0) + 2 * (1 - t) * t * np.array(P1) + t**2 * np.array(P2)
        curve.append(point.tolist())
    return curve
