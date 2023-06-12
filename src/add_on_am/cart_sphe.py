from math import sin, cos, atan2, sqrt, pi


def distance(a, b):
    """Returns the distance between two cartesian points."""
    x = (b[0] - a[0]) ** 2
    y = (b[1] - a[1]) ** 2
    z = (b[2] - a[2]) ** 2
    return (x + y + z) ** 0.5


def magnitude(x, y, z):
    """Returns the magnitude of the vector."""
    return sqrt(x * x + y * y + z * z)


def to_spherical(x, y, z):
    """Converts a cartesian coordinate (x, y, z) into a spherical one (radius, theta, phi)."""
    radius = magnitude(x, y, z)
    if z > 0:
        theta = atan2(sqrt(x * x + y * y), z)
    elif z < 0:
        theta = pi + atan2(sqrt(x * x + y * y), z)
    elif z == 0 and x*y != 0:
        theta = pi / 2
    else:
        theta = 0
    
    if x > 0:
        phi = atan2(y, x)
    elif x < 0 and y >= 0:
        phi = pi + atan2(y, x)
    elif x < 0 and y < 0:
        phi = -pi + atan2(y, x)
    elif x == 0 and y > 0:
        phi = pi / 2
    elif x == 0 and y < 0:
        phi = -pi / 2
    else:
        phi = 0
    return radius, theta, phi


def to_cartesian(radius, theta, phi):
    """Converts a spherical coordinate (radius, theta, phi) into a cartesian one (x, y, z)."""
    x = radius * cos(phi) * sin(theta)
    y = radius * sin(phi) * sin(theta)
    z = radius * cos(theta)
    return x, y, z


def shift_origin(x, y, z, origin):
    """Shifts the origin of a cartesian coordinate (x, y, z) to a new origin."""
    return x - origin[0], y - origin[1], z - origin[2]