"""Check rendered maps against known points, without hardware or a terminal."""

from pathlib import Path

text = Path("output/map-unit-frames.txt").read_text()
frames = ["LiDAR map:" + frame for frame in text.split("LiDAR map:")[1:]]
assert len(frames) == 4, "Refresh rate did not respect the 250 ms limit"
assert "Waiting for a complete rotation" in frames[0]


def grid(frame):
    return [line[1:-1] for line in frame.splitlines() if line.startswith("|")]


waiting = grid(frames[0])
assert len(waiting) == 41 and all(len(row) == 81 for row in waiting)
assert not any("*" in row for row in waiting), "Initial partial scan was displayed"

cardinal = grid(frames[1])
locations = {
    (row, column)
    for row, cells in enumerate(cardinal)
    for column, cell in enumerate(cells)
    if cell == "*"
}
assert locations == {(10, 40), (20, 60), (30, 40), (20, 20)}, locations
assert cardinal[20][40] == "R"
assert "Points: 4 visible / 6" in frames[1]
assert "Scale: 50 mm/column, 100 mm/row" in frames[1]

updated = grid(frames[2])
locations = {
    (row, column)
    for row, cells in enumerate(updated)
    for column, cell in enumerate(cells)
    if cell == "*"
}
assert locations == {(11, 40), (20, 34)}, locations
assert "Points: 2 visible / 2" in frames[2]
assert grid(frames[3]) == updated
assert "Scan age: 0.25 s" in frames[3]

report = (
    "PASS: cardinal directions, metric scale, and character aspect correction.\n"
    "PASS: incomplete initial rotation, zero returns, and out-of-range points omitted.\n"
    "PASS: completed scans replace old points; stale scan age remains visible.\n"
    "PASS: frames limited to 250 ms intervals.\n"
)
Path("output/map-unit-test.txt").write_text(report)
print(report, end="")
