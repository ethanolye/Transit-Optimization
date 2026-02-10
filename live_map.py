from flask import Flask, jsonify, render_template, request
import requests
import zipfile
import pandas as pd
import os
import xml.etree.ElementTree as ET
import logging
from datetime import datetime
from dotenv import load_dotenv

logging.basicConfig(level=logging.DEBUG, format="%(levelname)s: %(message)s")
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

TTC_VEHICLE_URL = (
    "https://webservices.umoiq.com/service/publicXMLFeed?command=vehicleLocations&a=ttc"
)


# =========================
# GTFS DATA LOADING
# =========================
print("\n=== Loading GTFS Data ===")

# Detect available GTFS files
GTFS_FILES = {}
for file in os.listdir("."):
    if file.endswith(".zip"):
        # Match various GTFS file patterns
        if "TTC" in file or "Routes and Schedules" in file:
            agency = "TTC"
        elif "GO" in file:
            agency = "GO"
        elif "UP" in file:
            agency = "UP"
        elif "YRT" in file or file.lower() == "google_transit.zip":
            continue
        else:
            continue

        GTFS_FILES[agency] = file
        print(f"Found {agency}: {file}")

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
                print(f"  ✓ Loaded shapes for {agency}")
            except KeyError:
                pass

            GTFS_DATA[agency] = {
                "routes": routes,
                "trips": trips,
                "stop_times": stop_times,
                "stops": stops,
                "shapes": shapes
            }
            print(f"  ✓ {agency}: {len(routes)} routes, {len(trips)} trips, {len(stops)} stops")
    except Exception as e:
        logging.error("Failed to load %s: %s", zip_file, e)

print(f"✓ GTFS Data loaded for: {', '.join(GTFS_DATA.keys())}\n")


def to_python_type(val):
    """Convert pandas/numpy types to Python native types"""
    if pd.isna(val):
        return None
    if hasattr(val, 'item'):  # numpy/pandas scalar
        return val.item()
    return val


def is_streetcar(route_number):
    """Check if route number is a streetcar (500-599 range)"""
    try:
        route_num = int(route_number)
        return 500 <= route_num <= 599
    except (ValueError, TypeError):
        return False


def time_to_seconds(t):
    h, m, s = map(int, t.split(":"))
    return h * 3600 + m * 60 + s


def format_gtfs_time(t):
    h, m, s = map(int, t.split(":"))
    day_offset = h // 24
    h = h % 24
    time_str = f"{h:02d}:{m:02d}"
    if day_offset > 0:
        return f"{time_str} +{day_offset}day"
    return time_str


def get_current_time_seconds():
    now = datetime.now()
    return now.hour * 3600 + now.minute * 60 + now.second


def get_trip_info(trip_id, route_id=None, preferred_agency=None):
    """
    Cross-reference a trip_id from the API with GTFS data.
    Falls back to route_id if trip_id lookup fails.
    Returns dict with route_name, destination, and route_id.
    """
    if not trip_id and not route_id:
        return None

    # Helper function to convert numpy types to Python native types
    def to_python_type(val):
        if pd.isna(val):
            return None
        if hasattr(val, 'item'):  # numpy/pandas scalar
            return val.item()
        return val

    # First, try to find by trip_id
    if trip_id:
        agencies = [preferred_agency] if preferred_agency in GTFS_DATA else list(GTFS_DATA.keys())
        for agency in agencies:
            data = GTFS_DATA[agency]
            trips = data["trips"]
            routes = data["routes"]

            # Find trip in GTFS - try both string and exact match
            trip_row = trips[trips['trip_id'] == trip_id]

            if trip_row.empty:
                continue

            # Get route info
            route_id = to_python_type(trip_row['route_id'].values[0])
            route_row = routes[routes['route_id'].astype(str) == str(route_id)]

            if not route_row.empty:
                route_name = to_python_type(route_row['route_long_name'].values[0])
                route_short = to_python_type(route_row['route_short_name'].values[0])

                # Get destination from trip_headsign, with fallback to route name
                destination = "Unknown"
                if 'trip_headsign' in trip_row.columns:
                    val = trip_row['trip_headsign'].values[0]
                    if pd.notna(val) and val != "":
                        destination = str(val).strip()

                # If still unknown, try to get from last stop
                if destination == "Unknown":
                    stop_times = data["stop_times"]
                    stops = data["stops"]
                    trip_stops = stop_times[stop_times['trip_id'] == trip_id].sort_values('stop_sequence')
                    if not trip_stops.empty:
                        last_stop_id = trip_stops.iloc[-1]['stop_id']
                        last_stop = stops[stops['stop_id'] == last_stop_id]
                        if not last_stop.empty:
                            destination = to_python_type(last_stop['stop_name'].values[0])

                # Determine vehicle type from GTFS route_type
                route_type = int(route_row['route_type'].values[0]) if 'route_type' in route_row.columns else 3
                # GTFS route types: 0=tram, 1=subway, 2=rail, 3=bus, 4=ferry, etc.
                vehicle_type = "train" if route_type in [0, 1, 2] else "bus"

                # Check if it's a streetcar (route 500-599)
                vehicle_subtype = "streetcar" if is_streetcar(route_short) else "bus"

                return {
                    "route_id": to_python_type(route_id),
                    "route_short_name": to_python_type(route_short),
                    "route_long_name": to_python_type(route_name),
                    "destination": destination,
                    "agency": agency,
                    "vehicle_type": vehicle_type,
                    "vehicle_subtype": vehicle_subtype
                }

    # Fallback: try to find by route_id alone
    if route_id:
        agencies = [preferred_agency] if preferred_agency in GTFS_DATA else list(GTFS_DATA.keys())
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

                # Check if it's a streetcar (route 500-599)
                vehicle_subtype = "streetcar" if is_streetcar(route_short) else "bus"

                return {
                    "route_id": to_python_type(route_id),
                    "route_short_name": to_python_type(route_short),
                    "route_long_name": to_python_type(route_name),
                    "destination": "See stops below",
                    "agency": agency,
                    "vehicle_type": vehicle_type,
                    "vehicle_subtype": vehicle_subtype
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
                    # Add stop_code if available
                    if 'stop_code' in row:
                        stop_data['stop_code'] = to_python_type(row['stop_code'])
                    result.append(stop_data)

                return result

        # Fallback: if no trip_id or trip not found, get stops for the route
        if route_id:
            route_trips = trips[trips['route_id'].astype(str) == str(route_id)]
            if not route_trips.empty:
                # Get first trip for this route to show stops
                first_trip_id = route_trips.iloc[0]['trip_id']
                trip_stops = stop_times[stop_times['trip_id'] == first_trip_id].sort_values('stop_sequence')

                if not trip_stops.empty:
                    trip_stops = trip_stops.merge(stops[['stop_id', 'stop_name', 'stop_lat', 'stop_lon']], on='stop_id')

                    result = []
                    for idx, row in trip_stops.iterrows():
                        result.append({
                            "stop_sequence": int(row['stop_sequence']),
                            "stop_id": row['stop_id'],
                            "stop_name": row['stop_name'],
                            "arrival_time": "N/A",
                            "departure_time": "N/A",
                            "lat": float(row['stop_lat']),
                            "lon": float(row['stop_lon'])
                        })

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

        # Fallback: if no trip found, try route_id
        if route_id:
            route_trips = trips[trips['route_id'].astype(str) == str(route_id)]
            if not route_trips.empty:
                # Get first trip for this route
                first_trip_id = route_trips.iloc[0]['trip_id']
                trip_row = trips[trips['trip_id'] == first_trip_id]

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


@app.route("/vehicles")
def vehicles():
    output = []

    # Fetch GO Transit vehicles
    try:
        if not API_KEY:
            raise ValueError("Missing GO API key")

        r = requests.get(
            GO_VEHICLE_URL,
            params={"key": API_KEY},
            timeout=5
        )
        r.raise_for_status()
        data = r.json()

        for entity in data.get("entity", []):
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
                    "vehicle_subtype": trip_info.get("vehicle_subtype", "bus")
                })
            else:
                # Default to bus if no GTFS match
                vehicle_data["agency"] = "GO"
                vehicle_data["vehicle_type"] = "bus"
                vehicle_data["vehicle_subtype"] = "bus"

            output.append(vehicle_data)
    except Exception as e:
        logging.error("Error fetching GO Transit vehicles: %s", e)

    # Fetch UP Express vehicles
    try:
        if not API_KEY:
            raise ValueError("Missing UP API key")

        r = requests.get(
            UP_VEHICLE_URL,
            params={"key": API_KEY},
            timeout=5
        )
        r.raise_for_status()
        data = r.json()

        for entity in data.get("entity", []):
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
                    "vehicle_subtype": trip_info.get("vehicle_subtype", "train")
                })
            else:
                vehicle_data["agency"] = "UP"
                vehicle_data["vehicle_type"] = "train"
                vehicle_data["vehicle_subtype"] = "train"

            output.append(vehicle_data)
    except Exception as e:
        logging.error("Error fetching UP Express vehicles: %s", e)

    # Fetch TTC vehicles
    try:
        r = requests.get(TTC_VEHICLE_URL, timeout=5)
        r.raise_for_status()
        root = ET.fromstring(r.content)

        for vehicle in root.findall(".//vehicle"):
            try:
                # TTC API returns data in attributes, not child elements
                trip_id = vehicle.get("tripTag")
                route_id = vehicle.get("routeTag")
                lat = float(vehicle.get("lat"))
                lon = float(vehicle.get("lon"))

                # Cross-reference with GTFS - pass route_id as fallback
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
                        "vehicle_subtype": trip_info.get("vehicle_subtype", "bus")
                    })
                else:
                    # Default TTC with streetcar detection based on route number (500-599)
                    vehicle_data["agency"] = "TTC"
                    vehicle_data["vehicle_type"] = "bus"
                    vehicle_data["vehicle_subtype"] = "streetcar" if is_streetcar(route_id) else "bus"

                output.append(vehicle_data)
            except (ValueError, TypeError, AttributeError):
                continue
    except Exception as e:
        logging.error("Error fetching TTC vehicles: %s", e)

    return jsonify(output)


@app.route("/api/go-transit-stop/<stop_code>")
def get_go_transit_predictions(stop_code):
    """Get GO Transit stop predictions for a specific stop"""
    try:
        GO_STOP_URL = "https://api.openmetrolinx.com/OpenDataAPI/api/V1/Stop/NextService"
        
        # Use API key in params
        params = {"key": API_KEY} if API_KEY else {}
        
        r = requests.get(
            f"{GO_STOP_URL}/{stop_code}",
            params=params,
            headers={'Accept': 'application/json'},
            timeout=5
        )
        r.raise_for_status()
        data = r.json()
        
        logging.debug(f"GO Transit API response for stop {stop_code}: {data}")
        
        # Extract predictions from the API response
        predictions = []
        
        # Check if NextService exists and has Lines
        next_service = data.get('NextService')
        if not next_service:
            logging.debug(f"No NextService data for stop {stop_code}")
            return jsonify({'predictions': [], 'stop_code': stop_code})
        
        lines = next_service.get('Lines', [])
        if not isinstance(lines, list):
            lines = [lines] if lines else []
        
        for line in lines:
            line_code = str(line.get('LineCode', ''))
            line_name = line.get('LineName', '')
            
            # Get departure times
            scheduled_time = line.get('ScheduledDepartureTime')
            computed_time = line.get('ComputedDepartureTime')
            
            # Skip if no times available
            if not scheduled_time and not computed_time:
                continue
            
            # Use computed time as prediction if available, otherwise use scheduled
            predicted_time = computed_time if computed_time else scheduled_time
            
            predictions.append({
                'route_id': line_code,
                'route_number': line_code,
                'line_name': line_name,
                'predicted_time': predicted_time,
                'scheduled_time': scheduled_time,
                'status': line.get('DepartureStatus', ''),
                'trip_number': line.get('TripNumber', '')
            })
        
        logging.debug(f"Parsed {len(predictions)} predictions for stop {stop_code}")
        return jsonify({'predictions': predictions, 'stop_code': stop_code})
        
    except requests.exceptions.HTTPError as e:
        logging.error(f"HTTP error fetching GO Transit stop {stop_code}: {e}")
        return jsonify({'predictions': [], 'error': str(e)}), 200
    except Exception as e:
        logging.error(f"Error fetching GO Transit stop predictions for {stop_code}: {e}")
        return jsonify({'predictions': [], 'error': str(e)}), 200


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


if __name__ == "__main__":
    app.run(debug=False)
