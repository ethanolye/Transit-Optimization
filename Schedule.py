import zipfile
import pandas as pd
from datetime import datetime, timedelta

# =========================
# Transit Agency Selector
# =========================
print("\nAvailable Transit Agencies:\n")
print("TTC - Toronto Transit Commission")
print("GO - GO Transit")
print("UP - UP Transit")
print("YRT - York Region Transit")

TAS = input("\nSelect Transit Agency: ").upper()

# Map user input to actual GTFS zip files
agency_files = {
    "TTC": "TTC/TTC Routes and Schedules Data.zip",
    "GO": "Metrolinx/GO-GTFS.zip",
    "UP": "Metrolinx/UP-GTFS.zip",
    "YRT": "google_transit.zip"
}

if TAS not in agency_files:
    raise RuntimeError(f"Unknown transit agency: {TAS}")

GTFS_ZIP = agency_files[TAS]
print(f"\nSelected GTFS file: {GTFS_ZIP}")

# =========================
# DEFAULT CONFIG
# =========================
ROUTE_ID = None
TARGET_DIRECTION = None
TARGET_STOP_SEQUENCE = None

# =========================
# DEBUG SETTINGS
# =========================
pd.set_option('display.max_rows', None)

# =========================
# TIME HELPERS
# =========================
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
    """Get current time in seconds since midnight"""
    now = datetime.now()
    return now.hour * 3600 + now.minute * 60 + now.second

def is_weekend():
    """Check if today is Saturday or Sunday"""
    return datetime.now().weekday() >= 5  # 5=Saturday, 6=Sunday

# =========================
# LOAD GTFS FILES
# =========================
print("\nLoading GTFS files...")
with zipfile.ZipFile(GTFS_ZIP) as z:
    routes = pd.read_csv(z.open("routes.txt"), low_memory=False)
    trips = pd.read_csv(z.open("trips.txt"), low_memory=False)
    stop_times = pd.read_csv(z.open("stop_times.txt"), low_memory=False)
    stops = pd.read_csv(z.open("stops.txt"), low_memory=False)

# =========================
# ROUTE SELECTION
# =========================
print("\nAvailable routes:\n")
print(routes[['route_id', 'route_short_name', 'route_long_name']].to_string(index=False))

while True:
    ROUTE_ID = input("\nEnter route_id to view: ").strip()
    if ROUTE_ID in routes['route_id'].astype(str).values:
        break
    print("Invalid route_id. Try again.")

# =========================
# GET ALL AVAILABLE DIRECTIONS FOR THIS ROUTE
# =========================
route_only_trips = trips[trips['route_id'].astype(str) == ROUTE_ID]
all_directions = sorted(route_only_trips['direction_id'].unique())

print(f"\nAvailable directions for Route {ROUTE_ID}:")
for direction in all_directions:
    headsigns = route_only_trips[route_only_trips['direction_id'] == direction]['trip_headsign'].unique()
    print(f"  Direction {direction}: {', '.join(headsigns)}")

# =========================
# DIRECTION SELECTION
# =========================
while True:
    try:
        TARGET_DIRECTION = int(input("\nEnter direction (0 or 1): ").strip())
        if TARGET_DIRECTION in all_directions:
            break
    except ValueError:
        pass
    print(f"Invalid direction. Available directions: {all_directions}")

# =========================
# GET STOPS FOR THIS ROUTE AND DIRECTION
# =========================
route_direction_trips = trips[
    (trips['route_id'].astype(str) == ROUTE_ID) &
    (trips['direction_id'] == TARGET_DIRECTION)
]

if route_direction_trips.empty:
    raise RuntimeError("No trips found for this route and direction.")

# Get all unique stops for this direction (across all trips)
# Pick the trip with the most stops (longest/most complete route for this direction)
trips_stop_counts = stop_times[stop_times['trip_id'].isin(route_direction_trips['trip_id'])].groupby('trip_id').size()
longest_trip_id = trips_stop_counts.idxmax()

direction_stops = stop_times[stop_times['trip_id'] == longest_trip_id].merge(
    stops[['stop_id', 'stop_name']], on='stop_id'
).sort_values('stop_sequence')[['stop_sequence', 'stop_id', 'stop_name']].drop_duplicates(subset=['stop_id']).reset_index(drop=True)

print(f"\nAvailable stops for this route and direction:")
print(direction_stops[['stop_sequence', 'stop_name']].to_string(index=False))

# =========================
# STATION SELECTION
# =========================
while True:
    try:
        TARGET_STOP_SEQUENCE = int(input("\nEnter stop_sequence to view: ").strip())
        if TARGET_STOP_SEQUENCE in direction_stops['stop_sequence'].values:
            break
    except ValueError:
        pass
    print("Invalid stop_sequence. Try again.")

TARGET_STOP_ID = direction_stops.loc[
    direction_stops['stop_sequence'] == TARGET_STOP_SEQUENCE, 'stop_id'
].values[0]

TARGET_STOP_NAME = direction_stops.loc[
    direction_stops['stop_sequence'] == TARGET_STOP_SEQUENCE, 'stop_name'
].values[0]
# =========================
# FETCH STOP TIMES FOR ALL TRIPS IN DIRECTION
# Include all platform stop_ids that share the same stop_name so branches
# that use different platform stop_ids at the same station are included.
# =========================
# find all stop_ids that match the selected stop name
stop_ids_for_name = stops.loc[stops['stop_name'] == TARGET_STOP_NAME, 'stop_id'].unique()

# filter station times to any of those stop_ids and only trips in the selected direction
station_times = stop_times[
    (stop_times['stop_id'].isin(stop_ids_for_name)) &
    (stop_times['trip_id'].isin(route_direction_trips['trip_id']))
].copy()

# annotate which platform(s) we included (useful for debugging / user info)
included_platforms = sorted(station_times['stop_id'].unique())
if len(included_platforms) > 1:
    print(f"\nNote: multiple platform stop_ids included for '{TARGET_STOP_NAME}': {included_platforms}")

station_times = station_times.merge(
    route_direction_trips[['trip_id', 'trip_headsign']], on='trip_id'
)

# Remove duplicates and sort by arrival
station_times = station_times.drop_duplicates(subset=['trip_headsign', 'arrival_time', 'departure_time'])
station_times['time_sec'] = station_times['arrival_time'].apply(time_to_seconds)
station_times = station_times.sort_values('time_sec')

# Filter to show only upcoming schedules
current_time_sec = get_current_time_seconds()
station_times = station_times[station_times['time_sec'] >= current_time_sec].copy()

# Format times for display
station_times['arrival_time'] = station_times['arrival_time'].apply(format_gtfs_time)
station_times['departure_time'] = station_times['departure_time'].apply(format_gtfs_time)

# =========================
# OUTPUT
# =========================
print("\n========================================")
print(f"Route ID: {ROUTE_ID}")
print(f"Station: {TARGET_STOP_NAME}")
print(f"Stop sequence: {TARGET_STOP_SEQUENCE}")
print(f"Direction: {TARGET_DIRECTION}")
print(f"Current time: {datetime.now().strftime('%H:%M:%S')}")
print(f"Day type: {'Weekend' if is_weekend() else 'Weekday'}")
print("========================================\n")

if station_times.empty:
    print("No upcoming schedules for today.")
else:
    for branch, group in station_times.groupby('trip_headsign'):
        print(f"Branch: {branch}")
        print(group[['arrival_time', 'departure_time']].reset_index(drop=True))
        print("-" * 40)
