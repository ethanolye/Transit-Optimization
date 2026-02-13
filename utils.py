"""Shared utility functions for transit data processing"""

import pandas as pd
import re
from datetime import datetime


def to_python_type(val):
    """Convert pandas/numpy types to Python native types"""
    if pd.isna(val):
        return None
    if hasattr(val, 'item'):  # numpy/pandas scalar
        return val.item()
    return val


def time_to_seconds(t):
    """Convert HH:MM:SS time string to seconds since midnight"""
    h, m, s = map(int, t.split(":"))
    return h * 3600 + m * 60 + s


def format_gtfs_time(t):
    """Format GTFS time string for display (handles times > 24h)"""
    h, m, s = map(int, t.split(":"))
    day_offset = h // 24
    h = h % 24
    time_str = f"{h:02d}:{m:02d}"
    if day_offset > 0:
        return f"{time_str} +{day_offset}day"
    return time_str


def get_current_time_seconds():
    """Get current time in seconds since midnight"""
    now = datetime.now()
    return now.hour * 3600 + now.minute * 60 + now.second


def is_weekend():
    """Check if today is Saturday or Sunday"""
    return datetime.now().weekday() >= 5  # 5=Saturday, 6=Sunday


def is_streetcar(route_number):
    """Check if route number is a streetcar (500-599 range)"""
    try:
        route_num = int(route_number)
        return 500 <= route_num <= 599
    except (ValueError, TypeError):
        return False


def extract_trip_details(trip_id_str):
    """
    Extract trip number and route from GTFS trip_id string.
    
    Supports multiple formats:
    - GO Transit: YYYYMMDD-route-tripnumber (3 parts)
    - UP Express: YYYYMMDD-tripnumber (2 parts)
    
    Returns: (trip_number, route_code)
    """
    trip_parts = str(trip_id_str).split('-')
    
    if len(trip_parts) >= 3:
        # GO Transit format
        return trip_parts[-1], trip_parts[1]
    elif len(trip_parts) == 2:
        # UP Express format
        return trip_parts[1], None
    else:
        # Fallback: treat entire string as trip number
        return str(trip_id_str), None


def extract_branch_from_headsign(headsign, route_id):
    """
    Extract branch letter from trip headsign.
    
    Patterns:
    - "96B - DC Oshawa GO" -> branch "B"
    - "East - 54A Lawrence East" -> branch "A"
    - "East - 54 Lawrence East" -> no branch (None)
    - "41 - Pickering GO" -> no branch (None)
    """
    if not headsign or " - " not in headsign:
        return None
    
    parts = headsign.split(" - ")
    first_part = parts[0].strip()
    
    # Try extracting from first part (e.g., "96B")
    branch_match = re.search(r'(\d+)([A-Za-z]{1,2})$', first_part)
    if branch_match:
        return branch_match.group(2)
    
    # Check if first part is just a number (no branch)
    if re.match(r'^\d+$', first_part):
        return None
    
    # Try extracting from second part (e.g., "54A Lawrence")
    if len(parts) > 1:
        second_part = parts[1].strip()
        branch_match2 = re.match(r'^(\d+)([A-Za-z]{1,2})?\s+', second_part)
        if branch_match2:
            return branch_match2.group(2)  # May be None
    
    return None
