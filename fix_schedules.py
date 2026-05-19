import json
import glob

patterns = [
    r"C:\Daten\ETCSProjekt\cda-rail-repo\test\data\instances\atmos2023\*\timetable\schedules.json",
    r"C:\Daten\ETCSProjekt\cda-rail-repo\test\data\instances\ras\*\timetable\schedules.json",
]
files = []
for pattern in patterns:
    files.extend(glob.glob(pattern))

print(f"Found {len(files)} files")


def left_value(value):
    return value[0] if isinstance(value, (list, tuple)) else value

for fpath in files:
    with open(fpath, 'r', encoding='utf-8') as f:
        data = json.load(f)

    is_ras = "\\ras\\" in fpath
    changed = False
    for train_key, train in data.items():
        if is_ras:
            for key in ("t_0", "t_n"):
                if isinstance(train.get(key), (list, tuple)):
                    train[key] = train[key][0]
                    changed = True

        stops = train.get("stops")
        if not stops:
            continue

        for i, stop in enumerate(stops):
            if "end" in stop:
                begin = left_value(stop["begin"])
                end = left_value(stop["end"])
                duration = end - begin
                if is_ras:
                    duration = max(stop.get("min_stopping_time", 0), duration)

                new_stop = {
                    "begin": begin,
                    "duration": duration,
                    "station": stop["station"],
                }
                stops[i] = new_stop
                changed = True

    if changed:
        with open(fpath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)
        print(f"Updated: {fpath}")
    else:
        print(f"No changes: {fpath}")

print("Done.")
