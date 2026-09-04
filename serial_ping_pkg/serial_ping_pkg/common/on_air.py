"""On-air ASCII decimal-place catalog (acoustic wire only).

ROS messages stay full precision (``GeoPoint`` float64, ``Float32``, …). These
constants and formatters are the single source of truth for how numbers are
printed onto ``$G``, ``$B``, and ``TEL:`` payloads.

At 59 N, ``LATLON_DECIMALS=7`` is ~1 cm in latitude (~0.6 cm in longitude),
matching ``DEPTH_DECIMALS=2`` (1 cm) so the three position axes share one grain.
``RANGE_DECIMALS`` and ``SPEED_DECIMALS`` use the same centimetre scale. SVS is
not a position axis and stays one decimal place.
"""

LATLON_DECIMALS = 7   # degrees; ~1 cm at 59 N
DEPTH_DECIMALS = 2    # metres (1 cm)
HEADING_DECIMALS = 0  # integer degrees
RANGE_DECIMALS = 2    # metres (TWTT dist, 1 cm)
SVS_DECIMALS = 1      # m/s
SPEED_DECIMALS = 2    # m/s

# Relay depth field is three integer digits + '.' + DEPTH_DECIMALS.
DEPTH_INT_DIGITS = 3
DEPTH_MAX = 10 ** DEPTH_INT_DIGITS - 10 ** (-DEPTH_DECIMALS)  # 999.99
HEADING_MAX = 359


def format_latlon(lat, lon, decimals=None):
    """Format ``lat,lon`` at ``LATLON_DECIMALS`` (or ``decimals`` if given)."""
    d = LATLON_DECIMALS if decimals is None else int(decimals)
    return f'{float(lat):.{d}f},{float(lon):.{d}f}'


def format_depth(depth):
    """Format depth in metres at ``DEPTH_DECIMALS`` (no width padding)."""
    return f'{float(depth):.{DEPTH_DECIMALS}f}'


def format_depth_padded(depth):
    """Clamp depth to ``[0, DEPTH_MAX]`` and zero-pad to the relay field width.

    With the catalog defaults this is ``DDD.DD`` (6 characters).
    """
    val = max(0.0, min(float(depth), DEPTH_MAX))
    width = DEPTH_INT_DIGITS + 1 + DEPTH_DECIMALS
    return f'{val:0{width}.{DEPTH_DECIMALS}f}'


def format_heading(heading):
    """Clamp heading to ``[0, HEADING_MAX]`` and format as zero-padded integer degrees."""
    val = max(0, min(int(heading), HEADING_MAX))
    return f'{val:03d}'


def format_range(dist):
    """Format a range in metres at ``RANGE_DECIMALS``."""
    return f'{float(dist):.{RANGE_DECIMALS}f}'


def format_svs(svs):
    """Format sound velocity in m/s at ``SVS_DECIMALS``."""
    return f'{float(svs):.{SVS_DECIMALS}f}'


def format_speed(speed):
    """Format speed in m/s at ``SPEED_DECIMALS``."""
    return f'{float(speed):.{SPEED_DECIMALS}f}'
