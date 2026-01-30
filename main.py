import zipfile
import pandas as pd

# =========================
# CONFIG
# =========================
GTFS_ZIP = "gtfs.zip"   # make sure this file is in the same folder
SHOW_ROWS = 40          # how many schedule rows to print

# =========================
# LOAD GTFS FILES
# =========================
with zipfile.ZipFile(GTFS_ZIP) as z:
    routes = pd.read_csv(z.open("routes.txt"), low_memory=False)
    trips = pd.read_csv(z.open("trips.txt"), low_memory=False)
    stop_times = pd.read_csv(z.open("stop_times.txt"), low_memory=False)
    stops = pd.read_csv(z.open("stops.txt"), low_memory=False)

# =========================
# SANITY CHECK
# =========================
print("Loaded GTFS data:")
print(f"Routes: {len(routes)}")
print(f"Trips: {len(trips)}")
print(f"Stop times: {len(stop_times)}")
print(f"Stops: {len(stops)}")

# =========================
# PICK A ROUTE (SAFE DEFAULT)
# =========================
print("\nAvailable routes (sample):")
print(routes[['route_id', 'route_short_name', 'route_long_name']].head(10))

# Use the first route as a guaranteed valid test
route_id = routes.iloc[0]['route_id']
print(f"\nUsing route_id: {route_id}")

# =========================
# GET TRIPS FOR ROUTE
# =========================
route_trips = trips[trips['route_id'] == route_id]

if route_trips.empty:
    raise RuntimeError("No trips found for this route.")

print(f"Trips on route: {len(route_trips)}")

# =========================
# GET STOP TIMES (SCHEDULE)
# =========================
route_stop_times = stop_times[
    stop_times['trip_id'].isin(route_trips['trip_id'])
]

if route_stop_times.empty:
    raise RuntimeError("No stop times found for this route.")

print(f"Stop time entries: {len(route_stop_times)}")

# =========================
# MERGE STOP NAMES
# =========================
schedule = route_stop_times.merge(
    stops[['stop_id', 'stop_name']],
    on='stop_id',
    how='left'
)

# =========================
# SORT LIKE A REAL TIMETABLE
# =========================
schedule = schedule.sort_values(
    ['trip_id', 'stop_sequence']
)

# =========================
# PRINT SCHEDULE
# =========================
print("\nRoute schedule (sample):")
print(
    schedule[
        [
            'trip_id',
            'stop_sequence',
            'stop_name',
            'arrival_time',
            'departure_time'
        ]
    ].head(SHOW_ROWS)
)

# =========================
# OPTIONAL: ONE TRIP ONLY
# =========================
print("\nSingle-trip timetable:")
first_trip_id = schedule['trip_id'].iloc[0]

single_trip = schedule[schedule['trip_id'] == first_trip_id]

print(
    single_trip[
        ['stop_sequence', 'stop_name', 'arrival_time', 'departure_time']
    ]
)
