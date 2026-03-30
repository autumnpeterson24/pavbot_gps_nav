"""
Small script for testing square waypoint navigation
"""
import math

def offset_latlon(lat, lon, north_m, east_m):
    ''' Create the offset for waypoints for square testing '''
    dlat = north_m / 111111.0
    dlon = east_m / (111111.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon

def square_waypoints(lat0, lon0, side_m):
    ''' Create square waypoints'''
    P0 = (lat0, lon0)
    P1 = offset_latlon(lat0, lon0, side_m, 0)
    P2 = offset_latlon(*P1, 0, side_m)
    P3 = offset_latlon(*P2, -side_m, 0)
    P4 = offset_latlon(*P3, 0, -side_m)  # back to start
    return [P0, P1, P2, P3, P4]

lat0, lon0 = 34.61459500, -112.45072167

 # plug in coordinates
wps = square_waypoints(lat0, lon0, 8) # change to how many meters want side of square to be for path

for lat, lon in wps:
    print(f"\"{lat:.8f},{lon:.8f}\",") # put waypoints into gps_waypoints.launch.py