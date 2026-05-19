import json
import glob

pattern = r"C:\Daten\ETCSProjekt\cda-rail-repo\test\data\instances\atmos2023\*\timetable\schedules.json"
files = glob.glob(pattern)
print(f"Found {len(files)} files")

for fpath in files:
    with open(fpath, 'r', encoding='utf-8') as f:
        data = json.load(f)

    changed = False
    for train_key, train in data.items():
        stops = train.get("stops")
        if not stops:
            continue
        for i, stop in enumerate(stops):
            if "end" in stop:
                new_stop = {"begin": stop["begin"], "duration": stop["end"] - stop["begin"], "station": stop["station"]}
                stops[i] = new_stop
                changed = True

    if changed:
        with open(fpath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
        print(f"Updated: {fpath}")
    else:
        print(f"No changes: {fpath}")

print("Done.")
