default_min_x = 1e+308
default_max_x = -1e+308
default_min_y = 1e+308
default_max_y = -1e+308
default_min_z = 1e+308
default_max_z = -1e+308


def find_aabb(points: [float], result: [float]):
    # Max value of floats ensures that when iterating over points of the
    # bounding box, the points that comprise the box will be modified to
    # reflect the true min and max of the box
    min_x = default_min_x
    max_x = default_max_x

    min_y = default_min_y
    max_y = default_max_y

    min_z = default_min_z
    max_z = default_max_z

    for p in points:
        min_x = min(p[0], min_x)
        max_x = max(p[0], max_x)

        min_y = min(p[1], min_y)
        max_y = max(p[1], max_y)

        min_z = min(p[2], min_z)
        max_z = max(p[2], max_z)

    result.append(min_x)
    result.append(min_y)
    result.append(min_z)

    result.append(max_x)
    result.append(max_y)
    result.append(max_z)
