import math
import numpy as np


def gps_to_local(lat, lon, initial_lat, initial_lon):
    """Convert GPS coordinates to local coordinates.

    Args:
        lat (float): Current latitude
        lon (float): Current longitude
        initial_lat (float): Reference latitude
        initial_lon (float): Reference longitude

    Returns:
        tuple: (x, y) position in meters from reference point
    """
    earth_radius = 6371000.0  # meters
    d_lat = lat - initial_lat
    d_lon = lon - initial_lon
    lat_rad = math.radians(initial_lat)

    x = earth_radius * d_lon * math.cos(lat_rad)
    y = earth_radius * d_lat

    return x, y


def calculate_covariance_gps(hdop=1.0):
    """Calculate GPS position covariance based on HDOP.

    Args:
        hdop (float): Horizontal dilution of precision

    Returns:
        numpy.ndarray: 6x6 covariance matrix
    """
    base_accuracy = 2.5  # typical GPS accuracy

    accuracy = base_accuracy * hdop

    covariance = np.zeros((6, 6))
    covariance[0][0] = accuracy**2  # x variance
    covariance[1][1] = accuracy**2  # y variance
    covariance[2][2] = (accuracy * 1.5) ** 2  # z variance (typically less accurate)

    return covariance


def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion.

    Args:
        roll (float): Roll angle in radians
        pitch (float): Pitch angle in radians
        yaw (float): Yaw angle in radians

    Returns:
        tuple: (x, y, z, w) quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * sr - sy * sp * cr  # x
    q[1] = sy * cp * sr + cy * sp * cr  # y
    q[2] = sy * cp * cr - cy * sp * sr  # z
    q[3] = cy * cp * cr + sy * sp * sr  # w

    return q
