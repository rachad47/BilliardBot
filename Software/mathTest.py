import numpy as np
import math

def line_circle_intersection(pt1, pt2, pt3, r):
    x1, y1 = pt1
    x2, y2 = pt2
    x0, y0 = pt3
    
    # Handle vertical line separately
    if x2 == x1:
        # Equation of line is x = x1
        # (x1 - x0)^2 + (y - y0)^2 = r^2
        # => (y - y0)^2 = r^2 - (x1 - x0)^2
        a = 1
        b = -2 * y0
        c = y0**2 + (x1 - x0)**2 - r**2
        discriminant = b**2 - 4*a*c
        if discriminant < 0:
            return []
        else:
            y1_root = (-b + math.sqrt(discriminant)) / (2*a) + y0
            y2_root = (-b - math.sqrt(discriminant)) / (2*a) + y0
            return [(x1, y1_root), (x1, y2_root)]
    else:
        # Non-vertical line
        m = (y2 - y1) / (x2 - x1)
        c = y1 - m * x1
        # Substituting y = mx + c into circle equation:
        # (x - x0)^2 + (mx + c - y0)^2 = r^2
        # Expanding and rearranging into standard quadratic form:
        A = 1 + m**2
        B = 2*m*(c - y0) - 2*x0
        C = x0**2 + (c - y0)**2 - r**2

        discriminant = B**2 - 4*A*C
        if discriminant < 0:
            return []
        else:
            sqrt_disc = math.sqrt(discriminant)
            x1_root = (-B + sqrt_disc) / (2*A)
            x2_root = (-B - sqrt_disc) / (2*A)
            y1_root = m*x1_root + c
            y2_root = m*x2_root + c
            return [(x1_root, y1_root), (x2_root, y2_root)]


# Usage
pt1 = (3, 0)
pt2 = (0, -6)
pt3 = (5, -5)
radius = 5


pt1 = (5, 5)
pt2 = (0, 5)
pt3 = (0, 0)
radius = 5
intersections = line_circle_intersection(pt1, pt2, pt3, radius)
print("Intersection Points:", intersections)
