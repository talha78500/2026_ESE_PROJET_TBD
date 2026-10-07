"""Compare C replay output with the previously calculated capture reference."""

import csv
from pathlib import Path


def read_points(path):
    with path.open(newline="") as source:
        return [
            (float(row["angle_deg"]), float(row["distance_mm"]))
            for row in csv.DictReader(source)
        ]


output = Path("output")
reference = read_points(output / "lidar-all-measurements.csv")
actual = read_points(output / "lidar-c-measurements.csv")
assert len(actual) == len(reference), "Measurement counts differ"

max_angle_error = 0.0
nearby = []
reference_nearby = []
for index, ((angle, distance), (ref_angle, ref_distance)) in enumerate(
    zip(actual, reference)
):
    angle_error = abs((angle - ref_angle + 180.0) % 360.0 - 180.0)
    max_angle_error = max(max_angle_error, angle_error)
    assert angle_error < 0.001, f"Angle differs at sample {index}"
    assert distance == ref_distance, f"Distance differs at sample {index}"
    if 0.0 < distance < 1000.0:
        nearby.append((angle, distance))
    if 0.0 < ref_distance < 1000.0:
        reference_nearby.append((ref_angle, ref_distance))

assert len(nearby) == len(reference_nearby), "Under-1-m return counts differ"
with (output / "lidar-c-under-1m.csv").open("w", newline="") as destination:
    writer = csv.writer(destination)
    writer.writerow(["angle_deg", "distance_mm"])
    writer.writerows((f"{angle:.6f}", f"{distance:.2f}") for angle, distance in nearby)

report = (
    f"PASS: all {len(actual)} C measurements match the saved reference.\n"
    f"Maximum circular angle difference: {max_angle_error:.6f} degrees.\n"
    "Distances match exactly.\n"
    f"All {len(nearby)} nonzero returns below 1000 mm match.\n"
)
(output / "parser-comparison.txt").write_text(report)
print(report, end="")
