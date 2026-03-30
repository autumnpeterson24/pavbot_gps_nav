import math

def offset_waypoint(lat0_deg, lon0_deg, distance_m, heading_deg):
    """
    Create Waypoint for PAVBot
    """
    meters_per_deg_lat = 111111.0

    heading_rad = math.radians(heading_deg)
    lat0_rad = math.radians(lat0_deg)

    north_m = distance_m * math.cos(heading_rad)
    east_m  = distance_m * math.sin(heading_rad)

    dlat_deg = north_m / meters_per_deg_lat
    dlon_deg = east_m / (meters_per_deg_lat * math.cos(lat0_rad))

    lat_goal_deg = lat0_deg + dlat_deg
    lon_goal_deg = lon0_deg + dlon_deg

    return lat_goal_deg, lon_goal_deg




lat0 = float(input("Starting lat: "))
lon0 = float(input("Starting lon: "))
distance_m = float(input("Distance in meters: "))
heading_deg = float(input("Heading in degrees (0=N, 90=E, 180=S, 270=W): "))

lat_goal, lon_goal = offset_waypoint(lat0, lon0, distance_m, heading_deg)

print("\nWaypoint:")
print(f"Latitude:  {lat_goal:.8f}")
print(f"Longitude: {lon_goal:.8f}")