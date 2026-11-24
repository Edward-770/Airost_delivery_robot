"""Generate the AIROST concept assembly and its preview mesh, in millimetres."""
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "outputs"
sys.path.insert(0, str(HERE / "cad_deps"))
import cadquery as cq

ICE = "#71b9cf"
STEEL = "#60778f"
SILVER = "#c4ced8"
NAVY = "#24445d"
TIRE = "#252b39"
GREEN = "#2d8d71"
VIOLET = "#9068b9"
ORANGE = "#ec9940"
RED = "#c75067"
BLUE = "#4a84dc"
GOLD = "#e3ba57"
TEAL = "#24ac9b"

parts = []


def box(x, y, z, w, l, h, radius=0):
    block = cq.Workplane("XY").box(w, l, h, centered=(True, True, False))
    if radius:
        block = block.edges("|Z").fillet(radius)
    return block.translate((x, y, z)).val()


def cyl(x, y, z, r, length, axis=(0, 0, 1)):
    return cq.Solid.makeCylinder(r, length, cq.Vector(x, y, z), cq.Vector(*axis))


def slot(x, y, length, width, along_x=False):
    return (
        cq.Workplane("XY")
        .center(x, y)
        .slot2D(length, width, 0 if along_x else 90)
        .extrude(8)
        .translate((0, 0, 45))
        .val()
    )


def polygon(points, z, height):
    return cq.Workplane("XY").polyline(points).close().extrude(height).translate((0, 0, z)).val()


def round_bar(a, b, radius):
    start, end = cq.Vector(*a), cq.Vector(*b)
    delta = end - start
    return cq.Solid.makeCylinder(radius, delta.Length, start, delta.normalized())


def add(name, solid, color, group):
    assert solid.isValid() and solid.Volume() > 0, name
    parts.append((name, solid, color, group))


# The forward direction is +Y. The floor is Z=0. +X is robot right.
# Rev B Improvements:
# 1. Universal mounting slot pattern for unmeasured motors, caster, and sensors
# 2. Tucked-in colour sensor (X=+58 mm) so overall width stays <=175 mm (<200 mm start box)
# 3. Centered low-CoG under-slung battery pack between motors for 50/50 left-right balance
# 4. Removed obstructive upper cargo tray for full access to NUCLEO-F756ZG pins & ST-LINK
# 5. Elevated front ToF sensor above gripper pivot & compact front ball corral/gate

plate = box(0, 0, 47, 134, 170, 4, 9)
for x, y in [(-56, -49), (56, -49), (-56, 12), (56, 12),
             (-29, 72), (29, 72), (46, 72), (58, 72),
             (-12, -65), (12, -65), (-17, 65), (17, 65),
             (-17, 78), (17, 78), (-62, 58), (62, 58), (-42, 78.5)]:
    plate = plate.cut(cyl(x, y, 46, 1.65, 6))

# Universal adjustment slots for motors, rear caster, and board mounts
for x, y, length, width, along_x in [
    # Left & right motor bracket universal adjustment slots
    (-53, 6, 22, 3.6, False), (-53, 30, 22, 3.6, False),
    (-38, 6, 18, 3.6, False), (-38, 30, 18, 3.6, False),
    (53, 6, 22, 3.6, False), (53, 30, 22, 3.6, False),
    (38, 6, 18, 3.6, False), (38, 30, 18, 3.6, False),
    # Rear caster universal adjustment slots (fore-aft and lateral tuning)
    (-14, -65, 20, 3.6, False), (14, -65, 20, 3.6, False),
    (0, -56, 22, 3.6, True), (0, -74, 22, 3.6, True),
    # Electronics & battery strap slots
    (26, -23, 16, 3.6, False), (54, -23, 16, 3.6, False),
    (-35, -51, 14, 3.6, True), (19, -51, 14, 3.6, True),
    (-35, 40, 14, 3.6, True), (19, 40, 14, 3.6, True),
]:
    plate = plate.cut(slot(x, y, length, width, along_x))
add("Lower chassis 170 x 134 x 4", plate.clean(), ICE, "chassis")

# Representative matched encoders/gearmotors. Verify shafts and mounting pattern.
for xsign, label in [(-1, "Left"), (1, "Right")]:
    out = (xsign, 0, 0)
    wheel = cyl(68.5 * xsign, 18, 32.5, 32.5, 19, out)
    wheel = wheel.cut(cyl(68 * xsign, 18, 32.5, 22, 20, out))
    add(label + " 65 mm tyre", wheel, TIRE, "drive")
    hub = cyl(70.5 * xsign, 18, 32.5, 22, 15, out)
    hub = hub.cut(cyl(69.5 * xsign, 18, 32.5, 3.1, 17, out))
    for y, z in [(30, 32.5), (18, 44.5), (6, 32.5), (18, 20.5)]:
        hub = hub.cut(cyl(69.5 * xsign, y, z, 3.8, 17, out))
    add(label + " wheel hub", hub, STEEL, "drive")
    add(label + " hub end cap", cyl(85.5 * xsign, 18, 32.5, 6, 1.1, out), SILVER, "drive")
    add(label + " gearmotor envelope", cyl(22 * xsign, 18, 32.5, 12.5, 42, out), STEEL, "drive")
    add(label + " encoder housing", cyl(16 * xsign, 18, 32.5, 9, 6, out), TIRE, "drive")
    add(label + " output shaft", cyl(64 * xsign, 18, 32.5, 3, 22, out), SILVER, "drive")
    saddle = box(xsign * 50, 18, 25, 22, 31, 22).cut(
        cyl(xsign * 38, 18, 32.5, 12.5, 24, out)
    )
    add(label + " motor saddle concept", saddle, NAVY, "drive")

ball_caster = cq.Solid.makeSphere(10, cq.Vector(0, -65, 10), angleDegrees1=-90)
add("Rear caster ball", ball_caster, SILVER, "drive")
caster = box(0, -65, 7.5, 32, 28, 17.5, 4).cut(
    cq.Solid.makeSphere(10.2, cq.Vector(0, -65, 10), angleDegrees1=-90)
)
add("Rear caster housing", caster, NAVY, "drive")
# Modular stackable shim spacers (18 mm base + two 2 mm leveling shims)
add("Rear caster base spacer", box(0, -65, 25, 29, 24, 18, 4), STEEL, "drive")
add("Rear caster 2mm leveling shim A", box(0, -65, 43, 29, 24, 2, 4), GOLD, "drive")
add("Rear caster 2mm leveling shim B", box(0, -65, 45, 29, 24, 2, 4), ORANGE, "drive")

# Centered low-CoG under-deck battery cradle (mounted centered at X=0 under rear-mid deck)
add("Centered under-deck battery cradle", box(0, -24, 19, 36, 84, 3, 3), NAVY, "electronics")
for x in [-16.5, 16.5]:
    add(f"Battery cradle hanger {x}", box(x, -24, 22, 3, 76, 25, 1), STEEL, "electronics")
add("Motor and servo power pack envelope", box(0, -24, 22, 30, 80, 23, 4), VIOLET, "electronics")
add("Battery retaining strap", box(0, -24, 45, 34, 14, 1.8, 0.4), NAVY, "electronics")

# Controller and electronics envelopes on open top deck (no upper tray blocking access)
for y in [-51, 40]:
    add(f"Nucleo support rail {y}", box(-14, y, 51, 66, 5, 8, 1), STEEL, "electronics")
add("NUCLEO-F756ZG PCB", box(-14, -8, 59, 70, 133.34, 1.6, 3), GREEN, "electronics")
add("ST-LINK USB micro B", box(-21, -73, 60.6, 9, 7, 4, 0.5), SILVER, "electronics")
add("NUCLEO Ethernet jack", box(1, 50, 60.6, 17, 17, 13, 0.8), SILVER, "electronics")
for x in [-41, 13]:
    add(f"NUCLEO header {x}", box(x, 9, 60.6, 5, 91, 8, 0.5), TIRE, "electronics")
add("STM32F756ZG package", box(-14, 11, 60.6, 20, 20, 2, 0.7), TIRE, "electronics")
add("ST-LINK section", box(-10, -51, 60.6, 9, 9, 1.2, 0.4), TIRE, "electronics")

# Right-side peripheral electronics rail (Motor driver + 5V servo buck + ESP32)
add("Motor driver mounting pad", box(44, 16, 51, 26, 22, 3, 1), NAVY, "electronics")
add("Motor driver envelope", box(44, 16, 54, 29, 24, 2, 1), GOLD, "electronics")
add("Driver IC placeholder", box(44, 16, 56, 8, 8, 2, 0.4), TIRE, "electronics")
add("5V Servo buck regulator pad", box(44, -18, 51, 24, 36, 3, 1), NAVY, "electronics")
add("5V Servo buck regulator PCB", box(44, -18, 54, 22, 34, 4, 1), RED, "electronics")
add("Wireless module mounting pad", box(44, 46, 51, 26, 20, 3, 1), NAVY, "electronics")
add("ESP32 radio board envelope", box(44, 46, 54, 28, 24, 2, 1), GREEN, "electronics")
add("ESP32 radio shield", box(46, 46, 56, 16, 16, 2, 0.6), SILVER, "electronics")

# Downward line array, 5 mm aperture to floor.
# Tucked-in RGB side arm places optical centre at X=+58 mm (inside wheel track width!).
line_carrier = box(0, 72, 9, 66, 25, 3, 2)
for x in [-29, 29]:
    line_carrier = line_carrier.cut(cyl(x, 72, 8, 1.65, 5))
    add(f"Line sensor spacer {x}", cyl(x, 72, 12, 3, 35), STEEL, "line")
add("Line sensor carrier plate", line_carrier, NAVY, "line")
add("5 channel analog IR line board", box(0, 72, 6.5, 37, 20, 2.5, 1.2), TEAL, "line")
for i, x in enumerate([-16, -8, 0, 8, 16], 1):
    add(f"IR optical pair {i}", box(x, 75, 5, 4, 3.5, 1.5, 0.4), TIRE, "line")

rgb_arm = box(51, 72, 44, 34, 15, 3, 2)
rgb_arm = rgb_arm.cut(slot(52, 72, 18, 3.4, True).translate(cq.Vector(0, 0, -2)))
add("Adjustable colour sensor side arm", rgb_arm, RED, "rgb")
add("Colour pod upright", box(58, 72, 12, 8, 8, 32, 1), STEEL, "rgb")
add("Colour pod lower carrier", box(58, 72, 9, 25, 25, 3, 2), NAVY, "rgb")
add("TCS34725 RGB breakout", box(58, 72, 7.4, 20.44, 20.28, 1.6, 1), RED, "rgb")
hood = box(58, 72, 4.5, 18, 18, 2.9, 2).cut(box(58, 72, 4.4, 14, 14, 3.1, 1.1))
add("Colour sensor light shield", hood, TIRE, "rgb")
add("RGB optical receiver", box(58, 72, 6.6, 4, 4, 0.8, 0.4), SILVER, "rgb")
add("RGB illumination LED", box(63, 72, 6.3, 3, 3, 1.1, 0.5), GOLD, "rgb")

# Three outward-facing ToF modules. Front ToF is elevated so gripper never blocks it.
for x, y, z_base, direction, label in [
    (-38, 81.5, 63, (0, 1, 0), "Front"),
    (-65, 58, 51, (-1, 0, 0), "Left"),
    (65, 58, 51, (1, 0, 0), "Right"),
]:
    dx, dy, _ = direction
    post_h = (z_base - 51) + 21
    add(label + " ToF mounting post", box(x - 5*dx, y - 5*dy, 51, 7, 7, post_h, 1), NAVY, "tof")
    pcb = box(x, y, z_base + 6, 13 if dy else 2, 2 if dy else 13, 18, 0.7)
    spacer = box(x - 2.5*dx, y - 2.5*dy, z_base + 8, 8 if dy else 3.2, 3.2 if dy else 8, 11, 0.6)
    add(label + " ToF carrier", spacer, STEEL, "tof")
    add(label + " VL53L1X breakout", pcb, BLUE, "tof")
    face = box(x + 1.5*dx, y + 1.5*dy, z_base + 14, 6 if dy else 1, 1 if dy else 6, 4, 0.4)
    add(label + " ToF optical face", face, TIRE, "tof")

# Compact two-servo pickup (shortened reach by 20 mm so robot fits 200x200 mm box better)
for x in [-18, 18]:
    add(f"Lift pivot tower {x}", box(x, 72, 51, 5, 22, 27, 2), NAVY, "pickup")
add("Lift servo", box(8, 72, 51, 11.6, 23.6, 24, 1.5), ORANGE, "pickup")
add("Lift pivot", cyl(-22, 72, 67, 3, 44, (1, 0, 0)), SILVER, "pickup")
for x in [-12, 12]:
    add(f"Lift arm {x}", round_bar((x, 72, 67), (x, 96, 35), 3.5), STEEL, "pickup")
add("Claw crossmember", box(0, 96, 28, 52, 7, 13, 1.8), NAVY, "pickup")
add("Claw servo", box(0, 92, 41, 23.6, 11.6, 24, 1.4), ORANGE, "pickup")
for sign, label in [(-1, "Left"), (1, "Right")]:
    finger_outline = [(sign*x, y) for x, y in [(17, 98), (24, 98), (29, 108),
                                                (28, 122), (22.5, 129), (19.5, 126),
                                                (21, 120), (22, 110)]]
    add(label + " ball gripper jaw", polygon(finger_outline, 26, 10), ORANGE, "pickup")
    add(label + " gripper pivot pin", cyl(21*sign, 98, 36, 2, 5), SILVER, "pickup")
    add(label + " clevis link", round_bar((0, 94, 43), (21*sign, 98, 40), 1.35), SILVER, "pickup")
add("40 mm ping pong ball reference", cq.Solid.makeSphere(20, cq.Vector(0, 121, 31), angleDegrees1=-90), "#faba63", "ball")

# Export a named, coloured assembly and a mesh of the same solids for the preview.
assembly = cq.Assembly(name="AIROST dual mode robot concept")
mesh = []
for name, item, hex_color, group in parts:
    rgb = [int(hex_color[i:i+2], 16) / 255 for i in (1, 3, 5)]
    assembly.add(item, name=name, color=cq.Color(*rgb))
    verts, faces = item.tessellate(0.15, 0.18)
    mesh.append(dict(name=name, color=hex_color, group=group,
                     vertices=[[round(p, 5) for p in v.toTuple()] for v in verts], faces=faces))

step = OUT / "AIROST_Robot_Concept.step"
assembly.export(str(step))
readback = cq.importers.importStep(str(step))
assert len(readback.solids().vals()) == len(parts)
bounds = readback.val().BoundingBox()
size = (bounds.xlen, bounds.ylen, bounds.zlen)
assert 165 < size[0] < 185 and 215 < size[1] < 235 and 75 < size[2] < 95, size
(HERE / "robot_mesh.json").write_text(json.dumps(mesh, separators=(",", ":")), encoding="utf-8")
print(f"{len(parts)} valid solids re-opened. Overall size X/Y/Z = {size} mm")
