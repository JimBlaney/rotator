from functools import lru_cache
import math
import traceback as tb

R60 = math.radians(60)
R80 = math.radians(80)

EPSILON = 1e-10
def zeroify_small_values(fn):
    def almost_equal(v1, v2):
        return abs(v1-v2) < EPSILON
    
    def wrapper(*args, **kwargs):
        ret = fn(*args, **kwargs)
        try:
            ret = [
                0 if almost_equal(v, 0) else v
                for v in ret
            ]
        except:
            pass
        return ret
    return wrapper

def intercept_error(fn):
    def wrapper(*args, **kwargs):
        try:
            ret = fn(*args, **kwargs)
        except Exception as ex:
            print(f'{fn.__name__} {ex} ({args}, {kwargs})')
            raise ex
        return ret
    return wrapper

@lru_cache(maxsize=128)
@zeroify_small_values
@intercept_error
def solve(a, b, c, degrees=True):
    try:
        """ solves the interior angles of a spherical triangle """
        a = math.radians(a) if degrees else a
        b = math.radians(b) if degrees else b
        c = math.radians(c) if degrees else c
        
        A = math.acos((math.cos(a) - (math.cos(b) * math.cos(c))) / (math.sin(b) * math.sin(c)))
        B = math.acos((math.cos(b) - (math.cos(a) * math.cos(c))) / (math.sin(a) * math.sin(c)))
        C = math.acos((math.cos(c) - (math.cos(a) * math.cos(b))) / (math.sin(a) * math.sin(b)))

        return [
            math.degrees(A) if degrees else A,
            math.degrees(B) if degrees else B,
            math.degrees(C) if degrees else C,
        ]
    except ValueError as ve:
        print(f'domain error. a={a}, b={b}, c={c}, {degrees}')
        raise ve


@lru_cache(maxsize=256)
@zeroify_small_values
def sph2cart(r, az, el, degrees=True):
    az = math.radians(az) if degrees else az
    el = math.radians(el) if degrees else el

    x = math.cos(az) * math.cos(el) * r
    y = math.sin(az) * math.cos(el) * r
    z = math.sin(el) * r

    return [x, y, z]

@lru_cache(maxsize=256)
def cart2sph(x, y, z, degrees=True):

    r = math.sqrt(x**2 + y**2 + z**2)

    az = math.degrees(math.atan2(x, y))       # azimuth from North, clockwise
    el = math.degrees(math.asin(z / r))       # elevation above horizon

    # Normalize azimuth to [0, 360)
    if az < 0:
        az += 360

    if degrees:
        return az, el
    else:
        return math.radians(az), math.radians(el)

@lru_cache(maxsize=128)
def rpy_rotation_matrix(roll, pitch, yaw=0, degrees=True):
    
    # Convert degrees to radians
    roll = math.radians(roll) if degrees else roll
    pitch = math.radians(pitch) if degrees else pitch
    yaw = math.radians(yaw) if degrees else yaw

    # Precompute sin and cos
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    R = [
        [
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr
        ],
        [
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr
        ],
        [
            -sp,
            cp * sr,
            cp * cr
        ]
    ]
    return R

@lru_cache(maxsize=128)
def quat_rotation_matrix(az, el):
    
    halfpi = math.pi / 2

    axis = [math.cos(az + halfpi), math.sin(az + halfpi), 0]
    angle = halfpi - el
    
    rotvec = [angle * x for x in axis]
    
    rx, ry, rz = rotvec
    
    theta = math.sqrt(rx**2 + ry**2 + rz**2)
    
    if theta == 0:
        # Zero rotation -> Identity matrix
        return [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        ]
    else:
        # Normalize the rotation axis
        x = rx / theta
        y = ry / theta
        z = rz / theta
        
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        one_minus_cos = 1.0 - cos_t
        
        # Rodrigues' formula
        return [
                cos_t + x * x * one_minus_cos,
                x * y * one_minus_cos - z * sin_t,
                x * z * one_minus_cos + y * sin_t,
                0.0,
                y * x * one_minus_cos + z * sin_t,
                cos_t + y * y * one_minus_cos,
                y * z * one_minus_cos - x * sin_t,
                0.0,
                z * x * one_minus_cos - y * sin_t,
                z * y * one_minus_cos + x * sin_t,
                cos_t + z * z * one_minus_cos,
                0.0,
                0.0, 0.0, 0.0, 1.0
        ]

def rotate(p, R):
    x, y, z = p

    x_rot = R[0][0]*x + R[0][1]*y + R[0][2]*z
    y_rot = R[1][0]*x + R[1][1]*y + R[1][2]*z
    z_rot = R[2][0]*x + R[2][1]*y + R[2][2]*z

    return [x_rot, y_rot, z_rot]

def magnitude(p):
    return math.sqrt(
        sum([
            v**2
            for v in p
        ])
    )

def norm(v):
    mag = magnitude(v)
    return [
        v_ / mag
        for v_ in v
    ]

def angle_between_points(p1, p2, degrees=True):
    p1 = norm(p1)
    p2 = norm(p2)
    dot = dot_product(p1, p2)
    dot = max(-1.0, min(1.0, dot))
    rad = math.acos(dot)
    return round(math.degrees(rad), 2) if degrees else rad

def dot_product(a, b):
    return sum(x*y for x,y in zip(a,b))

def cross_product(a, b):
    return [a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]]

def vector_sub(a, b):
    return [x - y for x, y in zip(a, b)]

def direction_azimuth_elevation(A, B, degrees=True):
    A = norm(A)
    B = norm(B)

    dot_AB = dot_product(A, B)
    proj = vector_sub(B, [dot_AB * x for x in A])
    t = norm(proj)

    global_z = [0, 0, 1]
    if abs(dot_product(A, global_z)) > 0.999:
        global_z = [0, 1, 0]

    E = cross_product(A, global_z)
    E = norm(E)

    N = cross_product(E, A)
    N = norm(N)

    t_E = dot_product(t, E)
    t_N = dot_product(t, N)

    azimuth = math.atan2(t_E, t_N)

    return round(math.degrees(azimuth), 2) if degrees else azimuth

C_OFFSET = solve(R60, R80, R60, False)[-1]

BOTTOM_N = sph2cart(106, 0, -60)
TOP_N = sph2cart(106, 0, 20)

@lru_cache(maxsize=128)
@intercept_error
def solve_positions(az, el, degrees=True):
    az = -math.radians(az if az else 0)
    el = math.radians(el if el else 0)

    matrix = quat_rotation_matrix(az, el)
    R = [
        matrix[0:4],
        matrix[4:8],
        matrix[8:12],
        matrix[12:16]
    ]

    top_n = rotate(TOP_N, R)
    n_arc_rad = angle_between_points(BOTTOM_N, top_n, False)
    C_n = solve(R60, n_arc_rad, R60, False)[-1]
    C_n -= C_OFFSET
    az_n = direction_azimuth_elevation(BOTTOM_N, top_n, False)
    C_n -= az_n
    C_n *= -1

    bottom_e = sph2cart(106, 270, -60)
    top_e = rotate(sph2cart(106, 270, 20), R)
    e_arc_rad = angle_between_points(bottom_e, top_e, False)
    C_e = solve(R60, e_arc_rad, R60, False)[-1]
    C_e -= C_OFFSET
    az_e = direction_azimuth_elevation(bottom_e, top_e, False)
    C_e -= az_e
    C_e *= -1

    bottom_s = sph2cart(106, 180, -60)
    top_s = rotate(sph2cart(106, 180, 20), R)
    s_arc_rad = angle_between_points(bottom_s, top_s, False)
    C_s = solve(R60, s_arc_rad, R60, False)[-1]
    C_s -= C_OFFSET
    az_s = direction_azimuth_elevation(bottom_s, top_s, False)
    C_s -= az_s
    C_s *= -1

    bottom_w = sph2cart(106, 90, -60)
    top_w = rotate(sph2cart(106, 90, 20), R)
    w_arc_rad = angle_between_points(bottom_w, top_w, False)
    C_w = solve(R60, w_arc_rad, R60, False)[-1]
    C_w -= C_OFFSET
    az_w = direction_azimuth_elevation(bottom_w, top_w, False)
    C_w -= az_w
    C_w *= -1

    if degrees:
        return [
            round(math.degrees(C_n), 2),
            round(math.degrees(C_e), 2),
            round(math.degrees(C_s), 2),
            round(math.degrees(C_w), 2)
        ]
    else:
        return [C_n, C_e, C_s, C_w]

@intercept_error
def haversine(az1: float, el1: float, az2: float, el2: float, degrees: bool = True) -> float:
    
    d_lat = el1 - el2
    d_lon = az1 - az2

    if degrees:
        d_lat = math.radians(d_lat)
        d_lon = math.radians(d_lon)
        el1 = math.radians(el1)
        el2 = math.radians(el2)

    a = math.sin(d_lat/2)**2 + \
        math.sin(d_lon/2)**2 * \
        math.cos(el1) * math.cos(el2)
    
    c = 2 * math.asin(math.sqrt(a))

    if degrees:
        return math.degrees(c)
    else:
        return c

@intercept_error
def rotate_along_great_circle(A, D, theta, degrees=True):
    A = norm(A)
    D = norm(D)
    N = norm(cross_product(A, D))  # Great circle normal
    T = cross_product(N, A)             # Tangent direction

    if degrees:
        theta = math.radians(theta)

    cosθ = math.cos(theta)
    sinθ = math.sin(theta)

    # Rodrigues' rotation formula: P(θ) = A*cosθ + T*sinθ
    return [a * cosθ + t * sinθ for a, t in zip(A, T)]

# @lru_cache(maxsize=512)
@intercept_error
def points_along_great_circle(A, D, arc_steps_deg: float) -> list[float]:
    points = []
    for deg in arc_steps_deg:
        theta = math.radians(deg)
        p = rotate_along_great_circle(A, D, theta, False)
        points.append([106 * v for v in p])
    return points

@intercept_error
def get_positions(az1: float, el1: float, az2: float, el2: float, interval: int = 0.25) -> tuple[list[list[float]], float]:
    positions = []
    arc_distance = haversine(az1, el1, az2, el2)

    if arc_distance < EPSILON:
        return [], 0

    steps = int(arc_distance // interval) + 1
    step_length = arc_distance / steps

    p1 = sph2cart(106, 90-az1, el1)
    p2 = sph2cart(106, 90-az2, el2)

    distances = [step_length * i for i in range(steps)] + [arc_distance]

    points = points_along_great_circle(p1, p2, distances)

    azel = []
    for point in points:
        az, el = cart2sph(*point)
        azel.append([az, el])
        pose = solve_positions(az, el)
        positions.append(pose)

    positions.append(solve_positions(az2, el2))

    return positions, arc_distance

def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolate on the scale given by a to b, using t as the point on that scale.
    Examples
    --------
        50 == lerp(0, 100, 0.5)
        4.2 == lerp(1, 5, 0.8)
    """
    return (1 - t) * a + t * b