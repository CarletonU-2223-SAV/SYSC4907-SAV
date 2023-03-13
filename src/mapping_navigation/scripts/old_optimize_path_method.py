def optimize_path(path: List[Point]) -> List[Point]:
    new_path = [[path[0]]]
    for i in range(len(path) - 1):
        start_point = path[i]
        end_point = path[i + 1]
        interval = 25
        distance = find_edge_weight(start_point, end_point)
        num_points_straight = int(distance / interval)

        # create points along straight road
        points_straight = []
        for j in range(2, num_points_straight):
            x = start_point.x + j * interval * (end_point.x - start_point.x) / distance
            y = start_point.y + j * interval * (end_point.y - start_point.y) / distance
            points_straight.append(Point(x, y, RoadSegmentType.STRAIGHT))
        new_path.append(points_straight)

        # create points along intersection
        if len(path) > 2 and i < (len(path) - 2):
            num_points_curve = 5
            radius = 33

            points_curve = []
            for j in range(num_points_curve):
                theta = j * (-math.pi / 2 / num_points_curve)  # Calculate the angle
                x = end_point.x - 20 + radius * math.cos(theta)  # Calculate the x-coordinate
                y = end_point.y + 20 + radius * math.sin(theta)  # Calculate the y-coordinate
                points_curve.append(Point(x, y, RoadSegmentType.INTERSECTION))  # Add the point to the list
            new_path.append(points_curve)

    new_path.append([path[-1]])
    print(new_path)
    merged_list = []
    for sublist in new_path:
        merged_list.extend(sublist)

    return merged_list