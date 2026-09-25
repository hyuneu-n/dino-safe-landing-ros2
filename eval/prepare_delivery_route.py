#!/usr/bin/env python3
"""Append a named landing spot to the generated city route."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("destination_id")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    data = json.loads(args.manifest.read_text(encoding="utf-8"))
    spot = next((s for s in data["landing_spots"]
                 if s["id"] == args.destination_id), None)
    if spot is None:
        choices = ", ".join(s["id"] for s in data["landing_spots"])
        raise SystemExit(f"Unknown destination {args.destination_id!r}; choose one of: {choices}")
    if spot.get("requires_elevation_support"):
        raise SystemExit("This recorder currently supports ground-level destinations only")

    route = [list(map(float, p)) for p in data["waypoints_xy"]]
    destination = [float(spot["x"]), float(spot["y"])]
    if route[-1] != destination:
        route.append(destination)
    result = {
        "layout": data.get("layout"),
        "seed": data.get("seed"),
        "start_xy": route[0],
        "target_xy": destination,
        "waypoints_xy": route,
        "selected_destination": spot,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"id": spot["id"], "label": spot["label"],
                      "x": spot["x"], "y": spot["y"],
                      "waypoints": len(route)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
