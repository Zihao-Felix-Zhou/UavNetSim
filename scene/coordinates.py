import math

from scene.models import EnuPoint, GeoAnchor


METERS_PER_DEGREE_LATITUDE = 110574.0


def lat_lon_to_enu(latitude, longitude, anchor: GeoAnchor):
    meters_per_degree_longitude = math.cos(math.radians(anchor.latitude)) * 111320.0
    return EnuPoint(
        x=(longitude - anchor.longitude) * meters_per_degree_longitude,
        y=(latitude - anchor.latitude) * METERS_PER_DEGREE_LATITUDE,
        z=0.0,
    )

