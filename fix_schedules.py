import json
import glob
import os
import shutil

patterns = [
    r"C:\Daten\ETCSProjekt\cda-rail-repo\test\data\instances\atmos2023\*\timetable\schedules.json",
    r"C:\Daten\ETCSProjekt\cda-rail-repo\test\data\instances\ras\*\timetable\schedules.json",
    r"C:\Daten\ETCSProjekt\cda-rail-repo\test\data\instances\gen-po\*\timetable\schedules.json",
]
files = []
for pattern in patterns:
    files.extend(glob.glob(pattern))

print(f"Found {len(files)} files")


def left_value(value):
    return value[0] if isinstance(value, (list, tuple)) else value


def infer_network_name(instance_name):
    if "SimpleNetwork" in instance_name:
        return "SimpleNetwork"
    if "Stammstrecke" in instance_name:
        return "Stammstrecke"
    return None

for fpath in files:
    with open(fpath, 'r', encoding='utf-8') as f:
        data = json.load(f)

    is_ras = "\\ras\\" in fpath
    is_gen_po = "\\gen-po\\" in fpath
    use_tuple_format = is_ras or is_gen_po
    changed = False
    for train_key, train in data.items():
        if use_tuple_format:
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
                if use_tuple_format:
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

    if is_gen_po:
        instance_dir = os.path.dirname(os.path.dirname(fpath))
        for old_network_dir in (
            os.path.join(instance_dir, "timetable", "network"),
            os.path.join(instance_dir, "network"),
        ):
            if os.path.isdir(old_network_dir):
                shutil.rmtree(old_network_dir)
                print(f"Deleted directory: {old_network_dir}")

        instance_name = os.path.basename(instance_dir)
        network_name = infer_network_name(instance_name)
        if network_name is not None:
            network_json_path = os.path.join(instance_dir, "network.json")
            with open(network_json_path, 'w', encoding='utf-8') as f:
                json.dump({"network": network_name}, f, indent=2)
                f.write("\n")
            print(f"Wrote: {network_json_path}")
        else:
            print(f"Skipped network.json (unknown network type): {instance_dir}")

print("Done.")
