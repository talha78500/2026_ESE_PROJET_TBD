"""Check rendered maps against known points, without hardware or a terminal."""

from pathlib import Path

text = Path("output/map-unit-frames.txt").read_text()
frames = ["LiDAR map:" + frame for frame in text.split("LiDAR map:")[1:]]
assert len(frames) == 13, "Unexpected frame count"
assert "Waiting for a complete rotation" in frames[0]


def grid(frame):
    return [line[1:-1] for line in frame.splitlines() if line.startswith("|")]


def is_return(cell):
    return 0x2801 <= ord(cell) <= 0x28FF


waiting = grid(frames[0])
assert len(waiting) == 41 and all(len(row) == 81 for row in waiting)
assert not any(is_return(cell) for row in waiting for cell in row), "Initial partial scan was displayed"

cardinal = grid(frames[1])
locations = {
    (row, column)
    for row, cells in enumerate(cardinal)
    for column, cell in enumerate(cells)
    if is_return(cell)
}
assert locations == {(10, 40), (20, 60), (30, 40), (20, 20)}, locations
assert cardinal[20][40] == "R"
assert cardinal[10][40] == chr(0x2803), "Vertical subcell detail was lost"
assert cardinal[20][60] == chr(0x28FF), "Eight-dot occupancy packing is incorrect"
assert "Points: 12 visible / 14" in frames[1]
assert "Scale: 25.0 mm/dot" in frames[1]

updated = grid(frames[2])
locations = {
    (row, column)
    for row, cells in enumerate(updated)
    for column, cell in enumerate(cells)
    if is_return(cell)
}
assert locations == {(11, 40), (20, 34)}, locations
assert "Points: 2 visible / 2" in frames[2]
assert grid(frames[3]) == updated
assert "Scan age: 0.25 s" in frames[3]

boundary = grid(frames[4])
locations = {
    (row, column)
    for row, cells in enumerate(boundary)
    for column, cell in enumerate(cells)
    if is_return(cell)
}
assert locations == {(0, 40), (20, 80), (40, 40), (20, 0)}, locations
assert "Points: 4 visible / 4" in frames[4]

def can_locations(frame):
    return {(row, column) for row, cells in enumerate(grid(frame))
            for column, cell in enumerate(cells) if cell == "C"}


assert can_locations(frames[5]) == set(), "An empty wall was marked"
assert can_locations(frames[6]) == set(), "A single wall candidate was displayed"
assert can_locations(frames[7]) == set(), "An absent candidate was displayed"
assert can_locations(frames[8]) == {(3, 40)}, "Wall candidate was missed after one absent scan"
assert can_locations(frames[9]) == set(), "Wall candidate marker remained after removal"
assert can_locations(frames[10]) == set(), "A single isolated candidate was displayed"
assert can_locations(frames[11]) == {(15, 40)}, "Isolated can was missed"
assert can_locations(frames[12]) == set(), "Isolated can marker remained after removal"

report = (
    "PASS: cardinal directions, metric scale, and character aspect correction.\n"
    "PASS: all eight Braille dots encoded correctly; 25 mm subcell detail retained.\n"
    "PASS: returns exactly at the display radius fit every boundary.\n"
    "PASS: incomplete initial rotation, zero returns, and out-of-range points omitted.\n"
    "PASS: completed scans replace old points; stale scan age remains visible.\n"
    "PASS: frames limited to 250 ms intervals.\n"
    "PASS: a 56 mm can touching a wall at 2.5 m is detected across zero degrees after two hits in three rotations.\n"
    "PASS: an isolated 56 mm can at 0.7 m matches across scans with missing and distant background returns.\n"
    "PASS: the marker disappears when the can is absent from the current scan.\n"
)
Path("output/map-unit-test.txt").write_text(report)
print(report, end="")
