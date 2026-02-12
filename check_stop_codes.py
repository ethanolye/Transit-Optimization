"""
Script to check stop codes in GO Transit GTFS data
"""
import zipfile
import pandas as pd
import os

# Find GO Transit GTFS file
go_gtfs_file = None
for file in os.listdir("."):
    if file.endswith(".zip") and "GO" in file:
        go_gtfs_file = file
        break

if not go_gtfs_file:
    print("❌ No GO Transit GTFS file found")
    exit(1)

print(f"✓ Found GO Transit GTFS: {go_gtfs_file}\n")

# Load stops.txt
with zipfile.ZipFile(go_gtfs_file) as z:
    stops = pd.read_csv(z.open("stops.txt"), low_memory=False)

print(f"Total stops in GTFS: {len(stops)}")
print(f"Columns: {list(stops.columns)}\n")

# Check if stop_code column exists
if 'stop_code' not in stops.columns:
    print("❌ stop_code column does NOT exist in stops.txt")
    exit(1)

print("✓ stop_code column exists\n")

# Count stops with/without stop_code
stops_with_code = stops[stops['stop_code'].notna()]
stops_without_code = stops[stops['stop_code'].isna()]

print(f"Stops WITH stop_code: {len(stops_with_code)} ({len(stops_with_code)/len(stops)*100:.1f}%)")
print(f"Stops WITHOUT stop_code: {len(stops_without_code)} ({len(stops_without_code)/len(stops)*100:.1f}%)\n")

# Show sample stops with codes
print("Sample stops with stop_code:")
print("-" * 80)
sample = stops_with_code.head(10)[['stop_id', 'stop_code', 'stop_name']].to_string(index=False)
print(sample)
print()

# Show sample stops without codes (if any)
if len(stops_without_code) > 0:
    print("\nSample stops WITHOUT stop_code:")
    print("-" * 80)
    sample = stops_without_code.head(10)[['stop_id', 'stop_name']].to_string(index=False)
    print(sample)
    print()

# Check stop_code format
print("\nStop code format analysis:")
print("-" * 80)
sample_codes = stops_with_code['stop_code'].head(20).tolist()
print(f"Sample stop codes: {sample_codes}")
print(f"stop_code dtype: {stops['stop_code'].dtype}")

# Check if stop_codes are numeric or string
if stops_with_code['stop_code'].dtype == 'object':
    print("stop_code is stored as string/object")
else:
    print(f"stop_code is stored as numeric: {stops_with_code['stop_code'].dtype}")
