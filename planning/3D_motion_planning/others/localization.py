import numpy
import utm

def global_to_local(global_position, global_home):
    """
    Converts a geodetic coordinate (longitude, latitude, up) into a local NED (North-East-Down) coordinate
    """
    longitude, latitude, altitude_h = global_home
    (easting_h, northing_h, _, _) = utm.from_latlon(latitude, longitude)
    longitude, latitude, altitude_g = global_position
    (easting_g, northing_g, _, _) = utm.from_latlon(latitude, longitude)
    local_position = numpy.array([northing_g - northing_h, easting_g - easting_h, -(altitude_g - altitude_h)])

    return local_position

def local_to_global(local_position, global_home):
    """
    Converts a local NED (North-East-Down) coordinates into a geodetic coordinate (longitude, latitude, up)
    """
    local_position_easting, local_position_northing, local_position_up = local_position
    (latitude_h, longitude_h, altitude_h) = global_home
    (easting_h, northing_h, zone_number, zone_letter) = utm.from_latlon(longitude_h, latitude_h)
    easting_l = local_position_easting + easting_h
    northing_l = local_position_northing + northing_h
    (latitude_l, longitude_l) = utm.to_latlon(easting_l, northing_l, zone_number, zone_letter)
    global_position = numpy.array([longitude_l, latitude_l, -(local_position_up - altitude_h)])

    return global_position

def random_goal(north_range, east_range, down_range, global_home,
                start=(0,0,0), min_dist = 0):
    """
    Returns a random goal, expressed in NED format
    ((north, east, down)

    Args:
    north_range: tuple(min,max) all inclusive
    easth_range: tuple(min,max) all inclusive
    down_range: tuple(min,max) all inclusive
    global_home: global coordinates of the center of the NED map
    start: start location in NED format
    min_dist: integer representing the minimum distance between start and goal
    """
    dist = 0
    while dist < min_dist:
        ned_coord = (np.random.randint(north_range[0], north_range[1]+1),
        np.random.randint(east_range[0], east_range[1]+1),
        np.random.randint(down_range[0], down_range[1]+1))
        dist = np.linalg.norm(np.array(ned_coord) - np.array(start))
    return ned_coord

