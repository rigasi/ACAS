#This contains all additional utility functions for performing all calculations for the attack geometries
import math


def calculate_relative_speed_in_knots(speed1_knots, bearing1, speed2_knots, bearing2):
    """
    Calculate the relative speed between two aircraft, with all speeds given in and returned in knots.

    Args:
    speed1_knots (float): Speed of the first aircraft in knots.
    bearing1 (float): Bearing of the first aircraft in degrees from north.
    speed2_knots (float): Speed of the second aircraft in knots.
    bearing2 (float): Bearing of the second aircraft in degrees from north.

    Returns:
    float: Relative speed in knots.
    """
    # Convert bearings from degrees to radians for trigonometric functions
    bearing1_rad = math.radians(bearing1)
    bearing2_rad = math.radians(bearing2)
    
    # Convert speed from knots to km/h for vector calculation
    speed1_kmh = speed1_knots * 1.852
    speed2_kmh = speed2_knots * 1.852

    # Calculate the velocity components for the first aircraft in km/h
    v_x1 = speed1_kmh * math.sin(bearing1_rad)
    v_y1 = speed1_kmh * math.cos(bearing1_rad)
    
    # Calculate the velocity components for the second aircraft in km/h
    v_x2 = speed2_kmh * math.sin(bearing2_rad)
    v_y2 = speed2_kmh * math.cos(bearing2_rad)
    
    # Calculate the components of the relative velocity vector in km/h
    relative_v_x = v_x1 - v_x2
    relative_v_y = v_y1 - v_y2
    
    # Calculate the magnitude of the relative velocity vector in km/h
    relative_speed_kmh = math.sqrt(relative_v_x**2 + relative_v_y**2)
    
    # Convert km/h back to knots
    relative_speed_knots = relative_speed_kmh / 1.852
    return relative_speed_knots

def simple_haversine(lat1, lon1, lat2, lon2):
    """Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)."""
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])#

    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371  # Radius of Earth in kilometers.
    return c * r


def haversine(lat1, lon1, lat2, lon2, elev1, elev2):
    """Calculate the slant circle distance between two points 
    on the earth (specified in decimal degrees) and their elevation difference (in meters),
    with the conversion of elevation to meters inside the function."""
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula to calculate the great circle distance
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    r = 6371 * 1000  # Radius of Earth in meters
    horizontal_distance = c * r

    elevation_difference = elev2 - elev1

    # Calculate the slant range using the Pythagorean theorem
    slant_range = math.sqrt(horizontal_distance**2 + elevation_difference**2)
    return slant_range

def calculate_time_to_cover_distance(distance, speed):
    """Calculate the time needed to cover a distance at a given speed.
    Speed should be in km/h."""
    return distance / speed

def calculate_future_position(lat, lon, speed, heading, t):
    """Calculate future position given speed, heading, and time."""
    # Convert degrees to radians
    lat = math.radians(lat)
    lon = math.radians(lon)
    heading = math.radians(heading)

    # Convert speed from knots to km/h
    speed_kmh = speed * 1.852
    
    # Earth's radius in kilometers
    R = 6371
    
    # Distance covered in kilometers
    distance = speed_kmh * t
    
    # New latitude in radians
    new_lat = math.asin(math.sin(lat) * math.cos(distance / R) +
                        math.cos(lat) * math.sin(distance / R) * math.cos(heading))
    # New longitude in radians
    new_lon = lon + math.atan2(math.sin(heading) * math.sin(distance / R) * math.cos(lat),
                               math.cos(distance / R) - math.sin(lat) * math.sin(new_lat))
    # Convert back to degrees
    new_lat = math.degrees(new_lat)
    new_lon = math.degrees(new_lon)
    return (new_lat, new_lon)

def calculate_initial_ghost_position(lat, lon, distance, bearing):
    """Calculate future position given current position, distance, and bearing."""
    # Convert from degrees to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing)

    # Earth's radius in kilometers
    R = 6371

    # New latitude in radians
    new_lat = math.asin(math.sin(lat_rad) * math.cos(distance / R) +
                        math.cos(lat_rad) * math.sin(distance / R) * math.cos(bearing_rad))
    
    # New longitude in radians
    new_lon = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance / R) * math.cos(lat_rad),
                                   math.cos(distance / R) - math.sin(lat_rad) * math.sin(new_lat))

    # Convert back to degrees
    new_lat = math.degrees(new_lat)
    new_lon = math.degrees(new_lon)
    return new_lat, new_lon


def calculate_required_heading(lat1, lon1, lat2, lon2):
    """Calculate the heading required from the ghost to the target."""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    initial_heading = math.degrees(math.atan2(x, y))
    return (initial_heading + 360) % 360


def calculate_elevation_angle(lat1, lon1, alt1, lat2, lon2, alt2):
    """Calculate the elevation angle between two objects"""
    # Differences in coordinates
    lat_diff = lat2 - lat1  # difference in latitude
    lon_diff = lon2 - lon1  # difference in longitude
    
    # Convert differences in lat/lon to meters (approximate values)
    # Assuming each degree of latitude is approximately 111 km (110574 meters)
    # and longitude varies based on latitude
    lat_distance_m = lat_diff * 110574
    lon_distance_m = lon_diff * (111320 * math.cos(math.radians(lat1)))
    
    # Calculate the straight-line horizontal distance using Pythagoras theorem
    horizontal_distance_m = math.sqrt(lat_distance_m**2 + lon_distance_m**2)
    
    # Altitude difference in meters
    altitude_difference_m = alt2 - alt1
    
    # Calculate the elevation angle using atan2
    elevation_angle = math.atan2(altitude_difference_m, horizontal_distance_m)
    
    # Convert to degrees
    return math.degrees(elevation_angle)