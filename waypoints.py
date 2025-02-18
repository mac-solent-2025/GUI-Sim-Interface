"""Perform various calculations on waypoints."""

from geographiclib.geodesic import Geodesic

def verify_format(position):
    """
    Verify that coordinates are given in the correct format.
    
    Parameters:
        position (list): lat/long position of an object in the following format: [latitude, longitude]
    """

    if len(position) != 2:
        raise Exception(f"Coordinates were given in an unexpected format: {position}")

    lat = position[0]
    long = position[1]
    if lat < -180 or lat > 180 or long < -180 or long > 180:
        raise Exception(f"Coordinates were given in an unexpected format: {position}")
        

def get_bearing(position, target):
    """
    Calculate a bearing in decimal degrees between a position and a target.

    Parameters:
        position (list): lat/long position of an object in the following format: [latitude, longitude]
        target (list): lat/long position of an object in the following format: [latitude, longitude]

    All units must be decimal degrees.
    """
    verify_format(position)
    verify_format(target)
    # Calculate bearing in signed azimuth format (-180-180)
    brng = Geodesic.WGS84.Inverse(position[0], position[1], target[0], target[1])['azi1']
    # Convert to unsigned azimuth
    if brng < 0:
        brng += 360
    return brng

def get_distance(position, target):
    """
    Calculate the geodesic distance between two waypoints.
    
    Parameters:
        position (list): lat/long position of an object in the following format: [latitude, longitude]
        target (list): lat/long position of an object in the following format: [latitude, longitude]

    All units must be decimal degrees.
    """
    verify_format(position)
    verify_format(target)
    # Use the Inverse method to calculate the geodesic distance.
    result = Geodesic.WGS84.Inverse(position[0], position[1], target[0], target[1])
    distance = result['s12']  # 's12' is the distance in metres.
    return distance