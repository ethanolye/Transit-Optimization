from flask import Flask, jsonify, render_template, request
import requests
import zipfile
import pandas as pd
import os
import re
import xml.etree.ElementTree as ET
import logging
from datetime import datetime
from dotenv import load_dotenv
from utils import (to_python_type, time_to_seconds, format_gtfs_time, 
                   get_current_time_seconds, is_streetcar, extract_trip_details,
                   extract_branch_from_headsign)

# Try to import GTFS-realtime bindings for TTC
try:
    from google.transit import gtfs_realtime_pb2
    GTFS_RT_AVAILABLE = True
except ImportError:
    GTFS_RT_AVAILABLE = False
    logging.warning("gtfs-realtime-bindings not available. Install with: pip install gtfs-realtime-bindings")

# Configure logging with both console and file output
log_format = "%(asctime)s - %(levelname)s - %(message)s"
log_dir = "logs"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

log_file = os.path.join(log_dir, "debug.log")

# Clear logger handlers and reconfigure
logging.getLogger().handlers.clear()

logging.basicConfig(
    level=logging.DEBUG,
    format=log_format,
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)
logging.getLogger("werkzeug").setLevel(logging.DEBUG)

load_dotenv()  # Load environment variables from .env file

_raw_api_key = os.getenv("API_KEY", "").strip()
API_KEY = _raw_api_key.strip('"').strip("'") or "30026663"

# API URLs
GO_VEHICLE_URL = (
    "https://api.openmetrolinx.com/OpenDataAPI/"
    "api/V1/Gtfs/Feed/VehiclePosition"
)

UP_VEHICLE_URL = (
    "https://api.openmetrolinx.com/OpenDataAPI/"
    "api/V1/UP/Gtfs/Feed/VehiclePosition"
)

# TTC GTFS-RT API URLs
TTC_VEHICLE_URL = "https://bustime.ttc.ca/gtfsrt/vehicles"
TTC_TRIP_ALERTS_URL = "https://bustime.ttc.ca/gtfsrt/trips"

# Legacy NextBus URL (kept as fallback)
TTC_NEXTBUS_URL = "https://webservices.nextbus.com/service/publicXMLFeed?command=routeConfig&a=ttc&r="

# Cache for TTC route branch information (route_id -> {direction_tag -> branch_info})
TTC_ROUTE_CACHE = {}


# =========================
# GTFS DATA LOADING
# =========================
print("\n=== Loading GTFS Data ===")

# Detect available GTFS files
GTFS_FILES = {}

# Check TTC directory
if os.path.exists("TTC"):
    # Look for TTC.zip specifically
    ttc_zip = os.path.join("TTC", "TTC.zip")
    if os.path.exists(ttc_zip):
        GTFS_FILES["TTC"] = ttc_zip
        print(f"Found TTC: {ttc_zip}")
    else:
        # Fallback to any .zip file if TTC.zip doesn't exist
        for file in os.listdir("TTC"):
            if file.endswith(".zip"):
                GTFS_FILES["TTC"] = os.path.join("TTC", file)
                print(f"Found TTC: {os.path.join('TTC', file)}")
                break

# Check Metrolinx directory
if os.path.exists("Metrolinx"):
    for file in os.listdir("Metrolinx"):
        if file.endswith(".zip"):
            if "GO" in file or "GO-GTFS" in file:
                GTFS_FILES["GO"] = os.path.join("Metrolinx", file)
                print(f"Found GO: {os.path.join('Metrolinx', file)}")
            elif "UP" in file or "UP-GTFS" in file:
                GTFS_FILES["UP"] = os.path.join("Metrolinx", file)
                print(f"Found UP: {os.path.join('Metrolinx', file)}")

# Load GTFS data from available files
GTFS_DATA = {}

for agency, zip_file in GTFS_FILES.items():
    try:
        print(f"Loading {agency} from {zip_file}...")
        with zipfile.ZipFile(zip_file) as z:
            routes = pd.read_csv(z.open("routes.txt"), low_memory=False)
            trips = pd.read_csv(z.open("trips.txt"), low_memory=False)
            stop_times = pd.read_csv(z.open("stop_times.txt"), low_memory=False)
            stops = pd.read_csv(z.open("stops.txt"), low_memory=False)

            # Try to load shapes if available
            shapes = None
            try:
                shapes = pd.read_csv(z.open("shapes.txt"), low_memory=False)
                print(f"  [OK] Loaded shapes for {agency}")
            except KeyError:
                pass

            GTFS_DATA[agency] = {
                "routes": routes,
                "trips": trips,
                "stop_times": stop_times,
                "stops": stops,
                "shapes": shapes
            }
            print(f"  [OK] {agency}: {len(routes)} routes, {len(trips)} trips, {len(stops)} stops")
    except Exception as e:
        logging.error("Failed to load %s: %s", zip_file, e)

print(f"[OK] GTFS Data loaded for: {', '.join(GTFS_DATA.keys())}\n")

# =========================
# GTFS LOOKUP CACHE
# =========================
print("=== Building GTFS lookup caches ===")
TRIP_INFO_CACHE = {}

for agency, data in GTFS_DATA.items():
    trips = data["trips"]
    routes = data["routes"]
    
    # Build route lookup dict
    route_dict = routes.set_index('route_id').to_dict('index')
    
    for idx, trip_row in trips.iterrows():
        trip_id = trip_row['trip_id']
        route_id = trip_row['route_id']
        
        if route_id not in route_dict:
            continue
            
        route_info = route_dict[route_id]
        route_short = route_info.get('route_short_name')
        route_long = route_info.get('route_long_name')
        route_type = int(route_info.get('route_type', 3))
        
        vehicle_type = "train" if route_type in [0, 1, 2] else "bus"
        vehicle_subtype = "streetcar" if is_streetcar(route_short) else ("train" if vehicle_type == "train" else "bus")
        
        destination = "Unknown"
        route_branch = None
        
        if pd.notna(trip_row.get('trip_headsign')):
            headsign = str(trip_row['trip_headsign']).strip()
            destination = headsign
            route_branch = extract_branch_from_headsign(headsign, route_id)
        
        shape_id = trip_row.get('shape_id') if pd.notna(trip_row.get('shape_id')) else None
        
        # Store with string key for consistent lookups
        cache_key = str(trip_id)
        TRIP_INFO_CACHE[cache_key] = {
            "route_id": route_id,
            "route_short_name": route_short,
            "route_long_name": route_long,
            "destination": destination,
            "agency": agency,
            "vehicle_type": vehicle_type,
            "vehicle_subtype": vehicle_subtype,
            "route_branch": route_branch,
            "shape_id": shape_id
        }

print(f"[OK] Built lookup cache for {len(TRIP_INFO_CACHE)} trips\n")


# =========================
# TTC ROUTE BRANCH HELPERS
# =========================
def fetch_ttc_route_config(route_id):
    """
    Fetch TTC route configuration from NextBus API to extract valid branch information.
    Caches results to avoid repeated API calls.
    
    Returns: dict mapping direction_tag -> branch_info
    Example: {"52_0_52A": {"branch": "A", "name": "..."},
              "52_0_52B": {"branch": "B", "name": "..."}}
    """
    if route_id in TTC_ROUTE_CACHE:
        return TTC_ROUTE_CACHE[route_id]
    
    try:
        url = f"{TTC_NEXTBUS_URL}{route_id}"
        logging.debug(f"Fetching TTC route config for route {route_id}")
        
        r = requests.get(url, timeout=5)
        r.raise_for_status()
        root = ET.fromstring(r.content)
        
        branch_info = {}
        
        # Parse route directions/branches from NextBus XML
        for direction in root.findall(".//direction"):
            dir_tag = direction.get("tag", "")
            dir_name = direction.get("name", "")
            
            if dir_tag:
                # Extract branch letter from direction name if present
                # Examples: "52A - Downtown Loop", "52B - Kipling"
                branch_match = re.search(r'^(\d+)([A-Z])\s*-', dir_name)
                branch = branch_match.group(2) if branch_match else None
                
                branch_info[dir_tag] = {
                    "name": dir_name,
                    "branch": branch
                }
        
        TTC_ROUTE_CACHE[route_id] = branch_info
        logging.debug(f"Cached {len(branch_info)} directions for TTC route {route_id}")
        return branch_info
        
    except Exception as e:
        logging.warning(f"Failed to fetch TTC route config for {route_id}: {e}")
        return {}


def extract_ttc_branch(dir_tag, route_id):
    """
    Extract branch letter from TTC dirTag using simple pattern matching.
    Avoids blocking HTTP calls during live vehicle fetch.
    
    Args:
        dir_tag: TTC dirTag (e.g., "52_0_52A")
        route_id: TTC route ID (e.g., "52")
        
    Returns: branch letter (e.g., "A") or None
    """
    if not dir_tag or not route_id:
        return None
    
    # Use simple regex pattern - handles 99% of cases like "52_0_52A" -> "A"
    # Avoids making HTTP calls which would block the /vehicles endpoint
    parts = str(dir_tag).split("_")
    if len(parts) >= 3:
        variant = parts[2]
        # Check if variant is exactly routeNumber + 1 letter branch
        branch_match = re.match(rf'^{re.escape(str(route_id))}([A-Z])$', variant)
        if branch_match:
            return branch_match.group(1)
    
    return None





def get_trip_info(trip_id, route_id=None, preferred_agency=None):
    """
    Cross-reference a trip_id from the API with GTFS data using cached lookups.
    Falls back to route_id if trip_id lookup fails.
    Returns dict with route_name, destination, and route_id.
    """
    if not trip_id and not route_id:
        return None

    # Fast O(1) cache lookup by trip_id (convert to string for consistent lookup)
    if trip_id:
        cache_key = str(trip_id)
        if cache_key in TRIP_INFO_CACHE:
            cached_info = TRIP_INFO_CACHE[cache_key].copy()
            # If preferred_agency specified, only return if agency matches
            if preferred_agency and cached_info["agency"] != preferred_agency:
                pass  # Continue to fallback
            else:
                return cached_info

    # Fallback: try to find by route_id alone (less common, slower path)
    if route_id:
        if preferred_agency:
            agencies = [preferred_agency] if preferred_agency in GTFS_DATA else []
        else:
            agencies = list(GTFS_DATA.keys())
        
        for agency in agencies:
            data = GTFS_DATA[agency]
            routes = data["routes"]

            route_row = routes[routes['route_id'].astype(str) == str(route_id)]
            if not route_row.empty:
                route_name = to_python_type(route_row['route_long_name'].values[0])
                route_short = to_python_type(route_row['route_short_name'].values[0])

                # Determine vehicle type from GTFS route_type
                route_type = int(route_row['route_type'].values[0]) if 'route_type' in route_row.columns else 3
                vehicle_type = "train" if route_type in [0, 1, 2] else "bus"
                vehicle_subtype = "streetcar" if is_streetcar(route_short) else ("train" if vehicle_type == "train" else "bus")

                # Get route description/branch if available
                route_branch = None
                if 'route_desc' in route_row.columns:
                    branch_val = route_row['route_desc'].values[0]
                    if pd.notna(branch_val) and branch_val != "":
                        route_branch = to_python_type(branch_val)

                return {
                    "route_id": to_python_type(route_id),
                    "route_short_name": to_python_type(route_short),
                    "route_long_name": to_python_type(route_name),
                    "destination": "See stops below",
                    "agency": agency,
                    "vehicle_type": vehicle_type,
                    "vehicle_subtype": vehicle_subtype,
                    "route_branch": route_branch,
                    "shape_id": None  # No specific trip, so no shape
                }

    return None


def infer_agency_for_route(route_id):
    """Infer agency for a route_id by checking loaded GTFS routes (prefer UP then GO)."""
    if not route_id:
        return None
    for agency in ["UP", "GO"]:
        if agency not in GTFS_DATA:
            continue
        routes = GTFS_DATA[agency]["routes"]
        match = routes[routes['route_id'].astype(str) == str(route_id)]
        if not match.empty:
            return agency
    return None


def get_trip_stops(trip_id, route_id=None, preferred_agency=None):
    """
    Get all stops for a trip with arrival/departure times and coordinates.
    If trip_id not found, falls back to showing all stops on the route.
    """
    agencies = [preferred_agency] if preferred_agency in GTFS_DATA else list(GTFS_DATA.keys())
    for agency in agencies:
        data = GTFS_DATA[agency]
        stop_times = data["stop_times"]
        stops = data["stops"]
        trips = data["trips"]

        # First, try to find stop times for this trip
        if trip_id:
            trip_stops = stop_times[stop_times['trip_id'] == trip_id].sort_values('stop_sequence')

            if not trip_stops.empty:
                # Merge with stop information - include stop_code if available
                stop_cols = ['stop_id', 'stop_name', 'stop_lat', 'stop_lon']
                if 'stop_code' in stops.columns:
                    stop_cols.append('stop_code')
                trip_stops = trip_stops.merge(stops[stop_cols], on='stop_id')

                result = []
                for idx, row in trip_stops.iterrows():
                    stop_data = {
                        "stop_sequence": int(row['stop_sequence']),
                        "stop_id": to_python_type(row['stop_id']),
                        "stop_name": to_python_type(row['stop_name']),
                        "arrival_time": row['arrival_time'],
                        "departure_time": row['departure_time'],
                        "lat": float(row['stop_lat']),
                        "lon": float(row['stop_lon'])
                    }
                    # Determine stop_code for GO Transit API
                    # Bus stops: GTFS has stop_code like 108049, API expects 08049 (without leading 1)
                    # Train stations: GTFS has no stop_code, use stop_id directly (e.g., "UN", "WH")
                    if 'stop_code' in row and pd.notna(row['stop_code']):
                        # Use stop_code from GTFS, remove leading "10" if present
                        code_str = str(int(row['stop_code']))
                        if code_str.startswith('10'):
                            stop_data['stop_code'] = code_str[1:]  # Remove leading '1'
                        else:
                            stop_data['stop_code'] = code_str
                    else:
                        # No stop_code in GTFS, use stop_id (for train stations like "UN", "WH")
                        stop_data['stop_code'] = to_python_type(row['stop_id'])
                    result.append(stop_data)

                return result

        # Fallback: if no trip_id or trip not found, get stops for the route with branch matching
        if route_id:
            route_trips = trips[trips['route_id'].astype(str) == str(route_id)]
            if not route_trips.empty:
                # Try to get the trip_info to find the branch
                trip_info = get_trip_info(trip_id, route_id, agency)
                selected_trip_id = None
                
                # If we have branch information, try to find a trip with matching branch
                if trip_info and trip_info.get('route_branch'):
                    branch = trip_info['route_branch']
                    # Look for trips with this branch in their headsign
                    if 'trip_headsign' in route_trips.columns:
                        # Match trips that have the branch letter in their headsign
                        matching_trips = route_trips[
                            route_trips['trip_headsign'].str.contains(f'{route_id}{branch}', na=False, case=False, regex=False)
                        ]
                        if not matching_trips.empty:
                            selected_trip_id = matching_trips.iloc[0]['trip_id']
                
                # If no branch match, try to match by destination/headsign
                if not selected_trip_id and trip_info and trip_info.get('destination'):
                    destination = trip_info['destination']
                    if 'trip_headsign' in route_trips.columns:
                        # Look for trips with similar destination
                        matching_trips = route_trips[
                            route_trips['trip_headsign'].str.contains(destination, na=False, case=False, regex=False)
                        ]
                        if not matching_trips.empty:
                            selected_trip_id = matching_trips.iloc[0]['trip_id']
                
                # Last resort: use first trip for this route
                if not selected_trip_id:
                    selected_trip_id = route_trips.iloc[0]['trip_id']
                
                trip_stops = stop_times[stop_times['trip_id'] == selected_trip_id].sort_values('stop_sequence')

                if not trip_stops.empty:
                    # Merge with stop information - include stop_code if available
                    stop_cols = ['stop_id', 'stop_name', 'stop_lat', 'stop_lon']
                    if 'stop_code' in stops.columns:
                        stop_cols.append('stop_code')
                    trip_stops = trip_stops.merge(stops[stop_cols], on='stop_id')

                    result = []
                    for idx, row in trip_stops.iterrows():
                        stop_data = {
                            "stop_sequence": int(row['stop_sequence']),
                            "stop_id": row['stop_id'],
                            "stop_name": row['stop_name'],
                            "arrival_time": "N/A",
                            "departure_time": "N/A",
                            "lat": float(row['stop_lat']),
                            "lon": float(row['stop_lon'])
                        }
                        # Determine stop_code for GO Transit API
                        # Bus stops: GTFS has stop_code like 108049, API expects 08049 (without leading 1)
                        # Train stations: GTFS has no stop_code, use stop_id directly (e.g., "UN", "WH")
                        if 'stop_code' in row and pd.notna(row['stop_code']):
                            # Use stop_code from GTFS, remove leading "10" if present
                            code_str = str(int(row['stop_code']))
                            if code_str.startswith('10'):
                                stop_data['stop_code'] = code_str[1:]  # Remove leading '1'
                            else:
                                stop_data['stop_code'] = code_str
                        else:
                            # No stop_code in GTFS, use stop_id (for train stations like "UN", "WH")
                            stop_data['stop_code'] = row['stop_id']
                        result.append(stop_data)

                    return result

    return []


def get_trip_shape(trip_id, route_id=None, preferred_agency=None):
    """
    Get the actual route shape for a trip if available in shapes.txt.
    Returns list of [lat, lon] coordinates representing the actual route.
    Falls back to stops if shapes are not available or if only route_id provided.
    """
    agencies = [preferred_agency] if preferred_agency in GTFS_DATA else list(GTFS_DATA.keys())
    for agency in agencies:
        data = GTFS_DATA[agency]
        trips = data["trips"]
        shapes = data["shapes"]

        # Try to get shape from trip_id
        if trip_id:
            trip_row = trips[trips['trip_id'] == trip_id]
            if not trip_row.empty:
                # Check if shape_id exists and shapes are available
                if shapes is not None and 'shape_id' in trip_row.columns:
                    shape_id = trip_row['shape_id'].values[0]

                    # Get shape coordinates
                    shape_coords = shapes[shapes['shape_id'] == shape_id].sort_values('shape_pt_sequence')

                    if not shape_coords.empty:
                        result = []
                        for idx, row in shape_coords.iterrows():
                            result.append({
                                "lat": float(row['shape_pt_lat']),
                                "lon": float(row['shape_pt_lon'])
                            })
                        return result

        # Fallback: if no trip found, try route_id with branch matching
        if route_id:
            route_trips = trips[trips['route_id'].astype(str) == str(route_id)]
            if not route_trips.empty:
                # Try to get the trip_info to find the branch
                trip_info = get_trip_info(trip_id, route_id, agency)
                selected_trip_id = None
                
                # If we have branch information, try to find a trip with matching branch
                if trip_info and trip_info.get('route_branch'):
                    branch = trip_info['route_branch']
                    # Look for trips with this branch in their headsign
                    if 'trip_headsign' in route_trips.columns:
                        # Match trips that have the branch letter in their headsign
                        matching_trips = route_trips[
                            route_trips['trip_headsign'].str.contains(f'{route_id}{branch}', na=False, case=False, regex=False)
                        ]
                        if not matching_trips.empty:
                            selected_trip_id = matching_trips.iloc[0]['trip_id']
                
                # If no branch match, try to match by destination/headsign
                if not selected_trip_id and trip_info and trip_info.get('destination'):
                    destination = trip_info['destination']
                    if 'trip_headsign' in route_trips.columns:
                        # Look for trips with similar destination
                        matching_trips = route_trips[
                            route_trips['trip_headsign'].str.contains(destination, na=False, case=False, regex=False)
                        ]
                        if not matching_trips.empty:
                            selected_trip_id = matching_trips.iloc[0]['trip_id']
                
                # Last resort: use first trip for this route
                if not selected_trip_id:
                    selected_trip_id = route_trips.iloc[0]['trip_id']
                
                trip_row = trips[trips['trip_id'] == selected_trip_id]

                if shapes is not None and 'shape_id' in trip_row.columns:
                    shape_id = trip_row['shape_id'].values[0]
                    shape_coords = shapes[shapes['shape_id'] == shape_id].sort_values('shape_pt_sequence')

                    if not shape_coords.empty:
                        result = []
                        for idx, row in shape_coords.iterrows():
                            result.append({
                                "lat": float(row['shape_pt_lat']),
                                "lon": float(row['shape_pt_lon'])
                            })
                        return result

    # Fallback to stops if no shape available
    return get_trip_stops(trip_id, route_id, preferred_agency)


app = Flask(__name__)


@app.after_request
def log_non_200(response):
    if response.status_code != 200:
        app.logger.error("%s %s -> %s", request.method, request.path, response.status)
    return response


@app.route("/")
def index():
    return render_template("map.html")


@app.route("/schedule")
def schedule():
    return render_template("schedule.html")


@app.route("/api/agencies")
def api_agencies():
    """Return list of available agencies with friendly names"""
    agency_names = {
        "TTC": "TTC - Toronto Transit Commission",
        "GO": "GO - GO Transit",
        "UP": "UP - UP Express"
    }

    agencies = []
    for key in sorted(GTFS_DATA.keys()):
        agencies.append({
            "id": key,
            "name": agency_names.get(key, key)
        })

    return jsonify(agencies)


@app.route("/api/routes/<agency>")
def api_routes(agency):
    """Return routes for an agency - optimized for performance"""
    if agency not in GTFS_DATA:
        return jsonify({"error": "Agency not found"}), 404

    data = GTFS_DATA[agency]

    # Only select necessary columns to reduce data transfer and processing
    def convert_record(record):
        return {k: to_python_type(v) for k, v in record.items()}

    # Select only essential columns
    routes = [convert_record(r) for r in data["routes"][['route_id', 'route_short_name', 'route_long_name']].to_dict('records')]
    
    print(f"Returning {len(routes)} routes for {agency}")
    
    return jsonify({
        "routes": routes
    })


@app.route("/api/route-data/<agency>/<path:route_id>")
def api_route_data(agency, route_id):
    """Return route-specific GTFS data for an agency - optimized for performance"""
    if agency not in GTFS_DATA:
        return jsonify({"error": "Agency not found"}), 404

    def convert_record(record):
        return {k: to_python_type(v) for k, v in record.items()}

    data = GTFS_DATA[agency]
    trips_df = data["trips"][['trip_id', 'route_id', 'direction_id', 'trip_headsign']]
    stop_times_df = data["stop_times"][['trip_id', 'stop_id', 'arrival_time', 'departure_time', 'stop_sequence']]
    stops_df = data["stops"][['stop_id', 'stop_name', 'stop_lat', 'stop_lon']]

    route_trips = trips_df[trips_df['route_id'].astype(str) == str(route_id)]
    trip_ids = route_trips['trip_id'].unique()
    route_stop_times = stop_times_df[stop_times_df['trip_id'].isin(trip_ids)]
    stop_ids = route_stop_times['stop_id'].unique()
    route_stops = stops_df[stops_df['stop_id'].isin(stop_ids)]

    trips = [convert_record(t) for t in route_trips.to_dict('records')]
    stop_times = [convert_record(st) for st in route_stop_times.to_dict('records')]
    stops = [convert_record(s) for s in route_stops.to_dict('records')]

    print(f"Returning route data for {agency} {route_id}: {len(trips)} trips, {len(stop_times)} stop_times, {len(stops)} stops")

    return jsonify({
        "trips": trips,
        "stop_times": stop_times,
        "stops": stops
    })


@app.route("/api/directions/<agency>/<path:route_id>")
def api_directions(agency, route_id):
    """Return available directions and headsigns for a route"""
    if agency not in GTFS_DATA:
        return jsonify({"error": "Agency not found"}), 404

    data = GTFS_DATA[agency]
    trips = data["trips"]

    route_trips = trips[trips['route_id'].astype(str) == str(route_id)]
    if route_trips.empty:
        return jsonify({"directions": []})

    directions = []
    for direction in sorted(route_trips['direction_id'].unique()):
        headsigns = route_trips[route_trips['direction_id'] == direction]['trip_headsign'].dropna().unique()
        directions.append({
            "direction_id": int(direction),
            "headsigns": [to_python_type(h) for h in headsigns]
        })

    return jsonify({"directions": directions})


@app.route("/api/stops/<agency>/<path:route_id>/<int:direction>")
def api_stops(agency, route_id, direction):
    """Return stops for a route + direction using longest trip logic"""
    if agency not in GTFS_DATA:
        return jsonify({"error": "Agency not found"}), 404

    data = GTFS_DATA[agency]
    trips = data["trips"]
    stop_times = data["stop_times"]
    stops = data["stops"]

    route_direction_trips = trips[
        (trips['route_id'].astype(str) == str(route_id)) &
        (trips['direction_id'] == direction)
    ]

    if route_direction_trips.empty:
        return jsonify({"stops": []})

    trips_stop_counts = stop_times[stop_times['trip_id'].isin(route_direction_trips['trip_id'])].groupby('trip_id').size()
    if trips_stop_counts.empty:
        return jsonify({"stops": []})

    longest_trip_id = trips_stop_counts.idxmax()

    direction_stops = stop_times[stop_times['trip_id'] == longest_trip_id].merge(
        stops[['stop_id', 'stop_name']], on='stop_id'
    ).sort_values('stop_sequence')[['stop_sequence', 'stop_id', 'stop_name']].drop_duplicates(subset=['stop_id']).reset_index(drop=True)

    results = []
    for _, row in direction_stops.iterrows():
        results.append({
            "stop_sequence": int(row['stop_sequence']),
            "stop_id": to_python_type(row['stop_id']),
            "stop_name": to_python_type(row['stop_name'])
        })

    return jsonify({"stops": results})


@app.route("/api/schedule/<agency>/<path:route_id>/<int:direction>/<int:stop_sequence>")
def api_schedule(agency, route_id, direction, stop_sequence):
    """Return upcoming schedules for a route, direction, and stop sequence"""
    if agency not in GTFS_DATA:
        return jsonify({"error": "Agency not found"}), 404

    data = GTFS_DATA[agency]
    routes = data["routes"]
    trips = data["trips"]
    stop_times = data["stop_times"]
    stops = data["stops"]

    route_direction_trips = trips[
        (trips['route_id'].astype(str) == str(route_id)) &
        (trips['direction_id'] == direction)
    ]

    if route_direction_trips.empty:
        return jsonify({"stop_name": None, "times": [], "platform_note": ""})

    trips_stop_counts = stop_times[stop_times['trip_id'].isin(route_direction_trips['trip_id'])].groupby('trip_id').size()
    if trips_stop_counts.empty:
        return jsonify({"stop_name": None, "times": [], "platform_note": ""})

    longest_trip_id = trips_stop_counts.idxmax()
    direction_stops = stop_times[stop_times['trip_id'] == longest_trip_id].merge(
        stops[['stop_id', 'stop_name']], on='stop_id'
    ).sort_values('stop_sequence')[['stop_sequence', 'stop_id', 'stop_name']].drop_duplicates(subset=['stop_id']).reset_index(drop=True)

    selected = direction_stops[direction_stops['stop_sequence'] == stop_sequence]
    if selected.empty:
        return jsonify({"stop_name": None, "times": [], "platform_note": ""})

    target_stop_name = selected.iloc[0]['stop_name']

    stop_ids_for_name = stops.loc[stops['stop_name'] == target_stop_name, 'stop_id'].unique()
    station_times = stop_times[
        (stop_times['stop_id'].isin(stop_ids_for_name)) &
        (stop_times['trip_id'].isin(route_direction_trips['trip_id']))
    ].copy()

    included_platforms = sorted(station_times['stop_id'].unique())
    platform_note = ""
    if len(included_platforms) > 1:
        platform_note = (
            f"<div class=\"platform-note\">"
            f"<i class=\"fas fa-info-circle\"></i>"
            f"<div class=\"text\">Note: Multiple platform stop IDs included for '{target_stop_name}': {included_platforms}</div>"
            f"</div>"
        )

    station_times = station_times.merge(
        route_direction_trips[['trip_id', 'trip_headsign']], on='trip_id'
    )

    station_times = station_times.drop_duplicates(subset=['trip_headsign', 'arrival_time', 'departure_time'])
    station_times['time_sec'] = station_times['arrival_time'].apply(time_to_seconds)
    station_times = station_times.sort_values('time_sec')

    current_time_sec = get_current_time_seconds()
    station_times = station_times[station_times['time_sec'] >= current_time_sec].copy()

    station_times['arrival_time'] = station_times['arrival_time'].apply(format_gtfs_time)
    station_times['departure_time'] = station_times['departure_time'].apply(format_gtfs_time)

    times = []
    for _, row in station_times.iterrows():
        times.append({
            "arrival_time": to_python_type(row['arrival_time']),
            "departure_time": to_python_type(row['departure_time']),
            "trip_headsign": to_python_type(row['trip_headsign'])
        })

    route_row = routes[routes['route_id'].astype(str) == str(route_id)]
    route_short = None
    route_long = None
    if not route_row.empty:
        route_short = to_python_type(route_row['route_short_name'].values[0])
        route_long = to_python_type(route_row['route_long_name'].values[0])

    return jsonify({
        "route_short_name": route_short,
        "route_long_name": route_long,
        "stop_name": to_python_type(target_stop_name),
        "times": times,
        "platform_note": platform_note
    })


@app.route("/api/stop-connections/<agency>/<path:stop_id>")
def api_stop_connections(agency, stop_id):
    """Return transfer connections (routes) available at a stop"""
    if agency not in GTFS_DATA:
        return jsonify({"error": "Agency not found"}), 404

    data = GTFS_DATA[agency]
    stop_times = data["stop_times"]
    trips = data["trips"]
    routes = data["routes"]

    trip_ids = stop_times[stop_times['stop_id'].astype(str) == str(stop_id)]['trip_id'].unique()
    if len(trip_ids) == 0:
        return jsonify({"routes": []})

    route_ids = trips[trips['trip_id'].isin(trip_ids)]['route_id'].unique()
    route_rows = routes[routes['route_id'].isin(route_ids)]

    results = []
    for _, row in route_rows.iterrows():
        results.append({
            "route_id": to_python_type(row.get('route_id')),
            "route_short_name": to_python_type(row.get('route_short_name')),
            "route_long_name": to_python_type(row.get('route_long_name'))
        })

    results.sort(key=lambda r: (str(r.get('route_short_name') or ""), str(r.get('route_long_name') or "")))

    return jsonify({"routes": results})


def fetch_metrolinx_predictions(stop_code, agency_name, requested_trip_id=None):
    """
    Shared helper to fetch predictions from Metrolinx API (GO Transit or UP Express).
    
    Args:
        stop_code: The stop code to query
        agency_name: "GO" or "UP" for logging
        requested_trip_id: Optional trip_id to filter by
        
    Returns:
        JSON response with predictions
    """
    METROLINX_STOP_URL = "https://api.openmetrolinx.com/OpenDataAPI/api/V1/Stop/NextService"
    
    logging.info(f"========================================")
    logging.info(f"Fetching {agency_name} predictions for stop_code: {stop_code}")
    if requested_trip_id:
        logging.info(f"  Filtering for trip_id: {requested_trip_id}")
    
    params = {"key": API_KEY} if API_KEY else {}
    full_url = f"{METROLINX_STOP_URL}/{stop_code}"
    
    try:
        r = requests.get(full_url, params=params, headers={'Accept': 'application/json'}, timeout=5)
        r.raise_for_status()
        data = r.json()
        
        next_service = data.get('NextService')
        if not next_service:
            logging.warning(f"[WARNING] No NextService data for stop {stop_code}")
            return jsonify({'predictions': [], 'stop_code': stop_code, 'message': 'No service data available for this stop'})
        
        lines = next_service.get('Lines', [])
        if not isinstance(lines, list):
            lines = [lines] if lines else []
        
        # Extract trip number and route from trip_id
        extracted_trip_number = None
        if requested_trip_id:
            extracted_trip_number, _ = extract_trip_details(requested_trip_id)
            logging.debug(f"Looking for trip number: '{extracted_trip_number}'")
        
        predictions = []
        exact_matches = []
        
        for line in lines:
            trip_number = str(line.get('TripNumber', ''))
            scheduled_time = line.get('ScheduledDepartureTime')
            computed_time = line.get('ComputedDepartureTime')
            
            if not scheduled_time and not computed_time:
                continue
            
            predicted_time = computed_time if computed_time else scheduled_time
            
            prediction_obj = {
                'route_id': str(line.get('LineCode', '')),
                'route_number': str(line.get('LineCode', '')),
                'line_name': line.get('LineName', ''),
                'predicted_time': predicted_time,
                'scheduled_time': scheduled_time,
                'status': line.get('DepartureStatus', ''),
                'trip_number': trip_number
            }
            
            # Check for exact trip match
            if requested_trip_id and trip_number == extracted_trip_number:
                logging.debug(f"  [MATCH] EXACT MATCH - Line {prediction_obj['route_id']}, Trip {trip_number}")
                exact_matches.append(prediction_obj)
            elif not requested_trip_id:
                logging.debug(f"  [OK] Line: {prediction_obj['route_id']}, Trip: {trip_number}")
                predictions.append(prediction_obj)
        
        # Use exact matches if found, otherwise use all
        if exact_matches:
            predictions = exact_matches
            logging.info(f"[OK] Found exact trip match for trip {extracted_trip_number}")
        elif requested_trip_id:
            logging.warning(f"[WARNING] No exact predictions for trip {extracted_trip_number} at stop {stop_code}")
        
        response_data = {'predictions': predictions, 'stop_code': stop_code}
        
        if predictions:
            trip_list = ", ".join([f"Trip {p['trip_number']}" for p in predictions])
            logging.info(f"[OK] Returning {len(predictions)} predictions for stop {stop_code}: {trip_list}")
        elif requested_trip_id:
            if lines:
                response_data['message'] = 'Trip not currently active at this stop'
                response_data['available_trips_count'] = len(lines)
            else:
                response_data['message'] = 'No service data available'
        else:
            response_data['message'] = 'No upcoming service'
        
        logging.info(f"========================================")
        return jsonify(response_data)
        
    except requests.exceptions.HTTPError as e:
        logging.error(f"HTTP error fetching {agency_name} predictions for stop {stop_code}: {e}")
        return jsonify({'predictions': [], 'error': str(e)}), 200
    except Exception as e:
        logging.error(f"Error fetching {agency_name} predictions for {stop_code}: {e}")
        return jsonify({'predictions': [], 'error': str(e)}), 200


@app.route("/vehicles")
def vehicles():
    from time import time as current_time
    output = []
    start_time = current_time()
    
    logging.info("=== Fetching vehicles from all agencies ===")

    # Fetch GO Transit vehicles
    try:
        logging.debug("Fetching GO Transit vehicles...")
        go_start = current_time()
        if not API_KEY:
            raise ValueError("Missing GO API key")

        r = requests.get(
            GO_VEHICLE_URL,
            params={"key": API_KEY},
            timeout=8
        )
        r.raise_for_status()
        data = r.json()
        
        go_count = 0
        for entity in data.get("entity", []):
            try:
                v = entity.get("vehicle")
                if not v or "position" not in v:
                    continue

                trip = v.get("trip", {})
                trip_id = trip.get("trip_id", "")
                route_id = trip.get("route_id", "")

                # Cross-reference with GTFS
                preferred_agency = infer_agency_for_route(route_id) or "GO"
                trip_info = get_trip_info(trip_id, route_id, preferred_agency) if (trip_id or route_id) else None
                if not trip_info and (trip_id or route_id):
                    trip_info = get_trip_info(trip_id, route_id)

                vehicle_data = {
                    "lat": float(v["position"]["latitude"]),
                    "lon": float(v["position"]["longitude"]),
                    "route": route_id,
                    "trip": trip_id
                }

                # Add GTFS cross-reference if found
                if trip_info:
                    vehicle_data.update({
                        "route_long_name": trip_info["route_long_name"],
                        "route_short_name": trip_info["route_short_name"],
                        "destination": trip_info["destination"],
                        "agency": trip_info["agency"],
                        "vehicle_type": trip_info["vehicle_type"],
                        "vehicle_subtype": trip_info.get("vehicle_subtype", "bus"),
                        "route_branch": trip_info.get("route_branch"),
                        "shape_id": trip_info.get("shape_id")
                    })
                else:
                    # Default to bus if no GTFS match
                    vehicle_data["agency"] = "GO"
                    vehicle_data["vehicle_type"] = "bus"
                    vehicle_data["vehicle_subtype"] = "bus"

                output.append(vehicle_data)
                go_count += 1
            except Exception as e:
                logging.debug(f"Skipping GO vehicle: {e}")
                continue
            
        elapsed = current_time() - go_start
        logging.info(f"[OK] GO Transit: {go_count} vehicles fetched in {elapsed:.2f}s")
    except Exception as e:
        logging.error(f"[ERROR] Error fetching GO Transit vehicles: {e}")

    # Fetch UP Express vehicles
    try:
        logging.debug("Fetching UP Express vehicles...")
        up_start = current_time()
        if not API_KEY:
            raise ValueError("Missing UP API key")

        r = requests.get(
            UP_VEHICLE_URL,
            params={"key": API_KEY},
            timeout=8
        )
        r.raise_for_status()
        data = r.json()
        
        up_count = 0
        for entity in data.get("entity", []):
            try:
                v = entity.get("vehicle")
                if not v or "position" not in v:
                    continue

                trip = v.get("trip", {})
                trip_id = trip.get("trip_id", "")
                route_id = trip.get("route_id", "")

                # Cross-reference with GTFS
                trip_info = get_trip_info(trip_id, route_id, "UP") if (trip_id or route_id) else None

                vehicle_data = {
                    "lat": float(v["position"]["latitude"]),
                    "lon": float(v["position"]["longitude"]),
                    "route": route_id,
                    "trip": trip_id
                }

                if trip_info:
                    vehicle_data.update({
                        "route_long_name": trip_info["route_long_name"],
                        "route_short_name": trip_info["route_short_name"],
                        "destination": trip_info["destination"],
                        "agency": trip_info["agency"],
                        "vehicle_type": trip_info["vehicle_type"],
                        "vehicle_subtype": trip_info.get("vehicle_subtype", "train"),
                        "route_branch": trip_info.get("route_branch"),
                        "shape_id": trip_info.get("shape_id")
                    })
                else:
                    vehicle_data["agency"] = "UP"
                    vehicle_data["vehicle_type"] = "train"
                    vehicle_data["vehicle_subtype"] = "train"

                output.append(vehicle_data)
                up_count += 1
            except Exception as e:
                logging.debug(f"Skipping UP vehicle: {e}")
                continue
            
        elapsed = current_time() - up_start
        logging.info(f"[OK] UP Express: {up_count} vehicles fetched in {elapsed:.2f}s")
    except Exception as e:
        logging.error(f"[ERROR] Error fetching UP Express vehicles: {e}")

    # Fetch TTC vehicles
    try:
        logging.debug("Fetching TTC vehicles...")
        ttc_start = current_time()
        
        if GTFS_RT_AVAILABLE:
            # Use new GTFS-RT API
            r = requests.get(TTC_VEHICLE_URL, timeout=8)
            r.raise_for_status()
            
            feed = gtfs_realtime_pb2.FeedMessage()
            feed.ParseFromString(r.content)
            
            ttc_count = 0
            logging.debug(f"TTC GTFS-RT API returned {len(feed.entity)} entities")
            
            for entity in feed.entity:
                if not entity.HasField('vehicle'):
                    continue
                    
                try:
                    vehicle = entity.vehicle
                    
                    # Extract vehicle position
                    if not vehicle.HasField('position'):
                        continue
                        
                    lat = vehicle.position.latitude
                    lon = vehicle.position.longitude
                    
                    # Extract trip and route information
                    trip_id = vehicle.trip.trip_id if vehicle.HasField('trip') and vehicle.trip.HasField('trip_id') else None
                    route_id = vehicle.trip.route_id if vehicle.HasField('trip') and vehicle.trip.HasField('route_id') else None
                    
                    # Cross-reference with GTFS
                    trip_info = get_trip_info(trip_id, route_id, "TTC") if (trip_id or route_id) else None

                    vehicle_data = {
                        "lat": lat,
                        "lon": lon,
                        "route": route_id,
                        "trip": trip_id
                    }

                    # Add GTFS cross-reference if found
                    if trip_info:
                        vehicle_data.update({
                            "route_long_name": trip_info["route_long_name"],
                            "route_short_name": trip_info["route_short_name"],
                            "destination": trip_info["destination"],
                            "agency": trip_info["agency"],
                            "vehicle_type": trip_info["vehicle_type"],
                            "vehicle_subtype": trip_info.get("vehicle_subtype", "bus"),
                            "route_branch": trip_info.get("route_branch"),
                            "shape_id": trip_info.get("shape_id")
                        })
                    else:
                        # Default TTC with streetcar detection based on route number (500-599)
                        vehicle_data["agency"] = "TTC"
                        vehicle_data["vehicle_type"] = "bus"
                        vehicle_data["vehicle_subtype"] = "streetcar" if is_streetcar(route_id) else "bus"

                    output.append(vehicle_data)
                    ttc_count += 1
                except (ValueError, TypeError, AttributeError) as e:
                    logging.debug(f"Skipping TTC vehicle: {e}")
                    continue
                    
            elapsed = current_time() - ttc_start
            logging.info(f"[OK] TTC: {ttc_count} vehicles fetched in {elapsed:.2f}s")
        else:
            logging.warning("[WARN] TTC vehicles skipped - gtfs-realtime-bindings not installed")
            
    except Exception as e:
        logging.error(f"[ERROR] Error fetching TTC vehicles: {e}")
    
    total_elapsed = current_time() - start_time
    logging.info(f"=== Total vehicles returned: {len(output)} (took {total_elapsed:.2f}s) ===")
    return jsonify(output)


@app.route("/api/go-transit-stop/<stop_code>")
def get_go_transit_predictions(stop_code):
    """Get GO Transit stop predictions for a specific stop, optionally filtered by trip_id"""
    requested_trip_id = request.args.get('trip_id', None)
    return fetch_metrolinx_predictions(stop_code, "GO Transit", requested_trip_id)


@app.route("/api/up-express-stop/<stop_code>")
def get_up_express_predictions(stop_code):
    """Get UP Express stop predictions for a specific stop, optionally filtered by trip_id"""
    requested_trip_id = request.args.get('trip_id', None)
    return fetch_metrolinx_predictions(stop_code, "UP Express", requested_trip_id)


@app.route("/api/shape/<agency>/<shape_id>")
def get_shape(agency, shape_id):
    """Get polyline coordinates for a shape_id (for route highlighting)"""
    if agency not in GTFS_DATA:
        return jsonify({"error": "Agency not found"}), 404

    data = GTFS_DATA[agency]
    shapes = data["shapes"]
    
    if shapes is None:
        return jsonify({"coordinates": []})
    
    shape_coords = shapes[shapes['shape_id'] == shape_id].sort_values('shape_pt_sequence')
    
    if shape_coords.empty:
        return jsonify({"coordinates": []})
    
    result = []
    for idx, row in shape_coords.iterrows():
        result.append({
            "lat": float(row['shape_pt_lat']),
            "lon": float(row['shape_pt_lon'])
        })
    
    return jsonify({
        "shape_id": shape_id,
        "agency": agency,
        "coordinates": result
    })


@app.route("/trip/<trip_id>")
def trip_details(trip_id):
    """Get stops, schedule, and shape for a specific trip."""
    # Extract route_id from query params if available
    route_id = request.args.get('route_id', None)
    agency = request.args.get('agency', None)

    stops = get_trip_stops(trip_id, route_id, agency)
    shape = get_trip_shape(trip_id, route_id, agency)
    info = get_trip_info(trip_id, route_id, agency)

    return jsonify({
        "trip_id": trip_id,
        "info": info,
        "stops": stops,
        "shape": shape
    })


@app.route("/api/ttc-trip-alerts")
def get_ttc_trip_alerts():
    """Get TTC trip updates and alerts from GTFS-RT API"""
    if not GTFS_RT_AVAILABLE:
        return jsonify({
            'error': 'GTFS-realtime bindings not installed',
            'message': 'Install with: pip install gtfs-realtime-bindings'
        }), 503
    
    # Optional filters
    route_id = request.args.get('route_id', None)
    trip_id = request.args.get('trip_id', None)
    
    try:
        logging.debug(f"Fetching TTC trip alerts (route={route_id}, trip={trip_id})")
        r = requests.get(TTC_TRIP_ALERTS_URL, timeout=8)
        r.raise_for_status()
        
        feed = gtfs_realtime_pb2.FeedMessage()
        feed.ParseFromString(r.content)
        
        alerts = []
        
        for entity in feed.entity:
            if not entity.HasField('trip_update'):
                continue
                
            trip_update = entity.trip_update
            
            # Extract trip information
            update_trip_id = trip_update.trip.trip_id if trip_update.HasField('trip') and trip_update.trip.HasField('trip_id') else None
            update_route_id = trip_update.trip.route_id if trip_update.HasField('trip') and trip_update.trip.HasField('route_id') else None
            
            # Apply filters
            if route_id and update_route_id != route_id:
                continue
            if trip_id and update_trip_id != trip_id:
                continue
            
            # Parse stop time updates
            stop_time_updates = []
            for stop_update in trip_update.stop_time_update:
                update_data = {
                    'stop_id': stop_update.stop_id if stop_update.HasField('stop_id') else None,
                    'stop_sequence': stop_update.stop_sequence if stop_update.HasField('stop_sequence') else None
                }
                
                # Arrival info
                if stop_update.HasField('arrival'):
                    arrival = stop_update.arrival
                    update_data['arrival'] = {
                        'delay': arrival.delay if arrival.HasField('delay') else None,
                        'time': arrival.time if arrival.HasField('time') else None
                    }
                
                # Departure info
                if stop_update.HasField('departure'):
                    departure = stop_update.departure
                    update_data['departure'] = {
                        'delay': departure.delay if departure.HasField('delay') else None,
                        'time': departure.time if departure.HasField('time') else None
                    }
                
                stop_time_updates.append(update_data)
            
            alert_data = {
                'entity_id': entity.id,
                'trip_id': update_trip_id,
                'route_id': update_route_id,
                'vehicle_id': trip_update.vehicle.id if trip_update.HasField('vehicle') and trip_update.vehicle.HasField('id') else None,
                'timestamp': trip_update.timestamp if trip_update.HasField('timestamp') else None,
                'stop_time_updates': stop_time_updates
            }
            
            alerts.append(alert_data)
        
        logging.info(f"[OK] TTC trip alerts: {len(alerts)} updates returned")
        
        return jsonify({
            'timestamp': feed.header.timestamp if feed.header.HasField('timestamp') else None,
            'alerts': alerts,
            'total_updates': len(alerts)
        })
        
    except requests.exceptions.RequestException as e:
        logging.error(f"Error fetching TTC trip alerts: {e}")
        return jsonify({'error': str(e)}), 500
    except Exception as e:
        logging.error(f"Error parsing TTC trip alerts: {e}")
        return jsonify({'error': str(e)}), 500


@app.route("/api/logs")
def view_logs():
    """Return debug logs as JSON with optional line limit"""
    lines = request.args.get('lines', default=100, type=int)
    
    if not os.path.exists(log_file):
        return jsonify({'error': 'Log file not found', 'logs': []})
    
    try:
        with open(log_file, 'r') as f:
            all_lines = f.readlines()
        
        # Return last N lines
        recent_lines = all_lines[-lines:] if len(all_lines) > lines else all_lines
        
        return jsonify({
            'total_lines': len(all_lines),
            'returned_lines': len(recent_lines),
            'logs': recent_lines
        })
    except Exception as e:
        return jsonify({'error': str(e), 'logs': []}), 500


@app.route("/logs")
def logs_page():
    """Display logs in a simple HTML page"""
    try:
        if not os.path.exists(log_file):
            return "<h1>Debug Logs</h1><p>Log file not found</p>"
        
        with open(log_file, 'r') as f:
            logs_content = f.read()
        
        # Convert to HTML with proper formatting
        logs_html = logs_content.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
        
        html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Debug Logs</title>
            <style>
                body {{ font-family: monospace; background: #f5f5f5; margin: 20px; }}
                .container {{ background: white; padding: 20px; border-radius: 5px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }}
                pre {{ background: #333; color: #0f0; padding: 15px; border-radius: 5px; overflow-x: auto; max-height: 600px; overflow-y: auto; }}
                .header {{ display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px; }}
                button {{ padding: 10px 20px; background: #007bff; color: white; border: none; border-radius: 4px; cursor: pointer; margin-right: 10px; }}
                button:hover {{ background: #0056b3; }}
                .info {{ font-size: 12px; color: #666; }}
            </style>
        </head>
        <body>
            <div class="container">
                <div class="header">
                    <h1>Debug Logs</h1>
                    <div>
                        <button onclick="location.reload()">Refresh</button>
                        <button onclick="downloadLogs()">Download</button>
                        <button onclick="clearLogs()">Clear Logs</button>
                    </div>
                </div>
                <p class="info">Last updated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</p>
                <pre>{logs_html}</pre>
            </div>
            
            <script>
                function downloadLogs() {{
                    fetch('/api/logs?lines=10000')
                        .then(res => res.json())
                        .then(data => {{
                            const content = data.logs.join('');
                            const blob = new Blob([content], {{ type: 'text/plain' }});
                            const url = window.URL.createObjectURL(blob);
                            const a = document.createElement('a');
                            a.href = url;
                            a.download = 'debug.log';
                            a.click();
                            window.URL.revokeObjectURL(url);
                        }});
                }}
                
                function clearLogs() {{
                    if (confirm('Are you sure you want to clear all logs?')) {{
                        fetch('/api/logs/clear', {{ method: 'POST' }})
                            .then(() => location.reload());
                    }}
                }}
                
                // Auto-refresh every 5 seconds
                setTimeout(() => location.reload(), 5000);
            </script>
        </body>
        </html>
        """
        return html
    except Exception as e:
        return f"<h1>Error</h1><p>{str(e)}</p>"


@app.route("/api/logs/clear", methods=['POST'])
def clear_logs():
    """Clear the debug log file"""
    try:
        if os.path.exists(log_file):
            open(log_file, 'w').close()
            logging.info("Debug logs cleared")
            return jsonify({'success': True, 'message': 'Logs cleared'})
        return jsonify({'success': False, 'message': 'Log file not found'}), 404
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


if __name__ == "__main__":
    app.run(debug=False)
