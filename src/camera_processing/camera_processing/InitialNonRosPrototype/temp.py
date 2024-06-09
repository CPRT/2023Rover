import numpy as np

def bounding_box(points: np.array):
    """
    Find min/max from an N-collection of coordinate pairs, shape = (N, 2), using 
    numpy's min/max along the collection-axis 
    """
    xMin = min(point[0] for point in points)
    yMin = min(point[1] for point in points)
    xMax = max(point[0] for point in points)
    yMax = max(point[1] for point in points)

    box = np.zeros((4, 2))

    box[0][0] = xMin
    box[0][1] = yMin

    box[1][0] = xMax
    box[1][1] = yMin

    box[2][0] = xMax
    box[2][1] = yMax

    box[3][0] = xMin
    box[3][1] = yMax

    return box


points = np.zeros((4, 2))
points[0][0] = 150
points[0][1] = 200

points[1][0] = 150
points[1][1] = 50

points[2][0] = 50
points[2][1] = 100

points[3][0] = 200
points[3][1] = 100

box = bounding_box(points)
print(f"Orig: {repr(points)}\nAfter: {repr(box)}")



