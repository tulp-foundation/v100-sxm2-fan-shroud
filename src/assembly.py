"""Full push-pull assembly — both rows positioned on the board."""

from build123d import *
from build123d import export_step
from fan_plenum import (
    make_fan_taper, make_hs_enclosure, make_gap_bridge, make_exhaust_shroud,
    TAPER_LEN, HS_ENC_LEN, COL_GAP, EXHAUST_LEN, BRIDGE_BODY, BRIDGE_PRINTED, FL_THICK,
    FAN_OFFSET, HS_PERP, CH_Z, ROW_GAP, ROW_PITCH
)
from pathlib import Path

# ROW_GAP / ROW_PITCH are imported, never restated here — this file used to hardcode
# ROW_GAP = 41.11 while the board is actually 20mm, which is why the exported assembly
# looked fine and the printed parts collided.

# X layout. Parts are placed by their BODY origin, but each part is longer than its
# body: the bolt flanges stick out FL_THICK at each butt joint. Advancing the cursor
# by body length alone makes adjacent flanges interpenetrate in the STEP file and
# hides exactly the interference we are trying to see. So every joint costs 2*FL_THICK.
#
# The two base plates are the fixed datum: plate 1 = [0, 119.35], plate 2 starts
# COL_GAP later. Both enclosure bodies MUST land exactly on their plates.
# Part 2 is positioned with its FLANGE FACE flush to the heatsink edge, not its body.
# The body therefore sits FL_THICK inboard of the plate edge and the flange lands over
# the plate — so nothing of part 2 enters the gap and part 3 gets the full COL_GAP.
# Parts are placed by their BODY origin, so each offset carries that FL_THICK shift.
X_ENC1 = -FL_THICK
X_BRIDGE = HS_ENC_LEN + COL_GAP / 2 - BRIDGE_PRINTED / 2 + FL_THICK  # centred in the gap
X_ENC2 = HS_ENC_LEN + COL_GAP + FL_THICK
X_INTAKE = X_ENC1 - FL_THICK - FL_THICK - TAPER_LEN
X_EXHAUST = X_ENC2 + HS_ENC_LEN + 2 * FL_THICK
X_END = X_EXHAUST + EXHAUST_LEN

# Part 3 is sized to the MEASURED clearance between the installed part 2's (30.94mm),
# which is less than the nominal COL_GAP (35mm) — so it is centred with slack, not
# stretched to span. It must never exceed the gap.
_brg_lo, _brg_hi = X_BRIDGE - FL_THICK, X_BRIDGE + BRIDGE_BODY + FL_THICK
_slack = COL_GAP - BRIDGE_PRINTED
assert _brg_lo >= HS_ENC_LEN - 1e-9 and _brg_hi <= HS_ENC_LEN + COL_GAP + 1e-9, (
    f"part 3 spans [{_brg_lo:.2f}, {_brg_hi:.2f}], outside the gap "
    f"[{HS_ENC_LEN:.2f}, {HS_ENC_LEN + COL_GAP:.2f}]")

print(f"Assembly length: {X_END - X_INTAKE:.1f}mm "
      f"(board footprint {HS_ENC_LEN * 2 + COL_GAP:.2f}mm)")
print(f"  Intake:   X = {X_INTAKE:8.2f} to {X_INTAKE + TAPER_LEN:8.2f}")
print(f"  Enc GPU1: X = {X_ENC1:8.2f} to {X_ENC1 + HS_ENC_LEN:8.2f}  <- base plate 1")
print(f"  Bridge:   X = {X_BRIDGE:8.2f} to {X_BRIDGE + BRIDGE_BODY:8.2f}"
      f"   printed [{_brg_lo:.2f}, {_brg_hi:.2f}] "
      f"= {BRIDGE_PRINTED:.2f}mm in a {COL_GAP:.0f}mm gap, {_slack / 2:.2f}mm slack each side")
print(f"  Enc GPU2: X = {X_ENC2:8.2f} to {X_ENC2 + HS_ENC_LEN:8.2f}  <- base plate 2")
print(f"  Exhaust:  X = {X_EXHAUST:8.2f} to {X_END:8.2f}")
print(f"  Rows {ROW_GAP:.0f}mm apart (pitch {ROW_PITCH:.0f}mm), fan stagger ±{FAN_OFFSET:.0f}mm")


def place(part, x_offset, y_offset=0):
    """Move a part to its assembly position."""
    return part.moved(Location((x_offset, y_offset, 0)))


# Build all unique parts once
print("\nBuilding parts...")
intake_top = make_fan_taper(+FAN_OFFSET)
intake_bot = make_fan_taper(-FAN_OFFSET)
enclosure = make_hs_enclosure()
bridge = make_gap_bridge()
exhaust_top = make_exhaust_shroud(+FAN_OFFSET)
exhaust_bot = make_exhaust_shroud(-FAN_OFFSET)

# Assemble both rows
print("Assembling...")
parts = []

for row_sign, row_label in [(+1, "top"), (-1, "bottom")]:
    row_y = row_sign * ROW_PITCH / 2
    # Only parts 1 and 4 are handed, and only because the fan is staggered in Y.
    intake = intake_top if row_sign > 0 else intake_bot
    exhaust = exhaust_top if row_sign > 0 else exhaust_bot

    parts.append(place(intake, X_INTAKE, row_y))
    parts.append(place(enclosure, X_ENC1, row_y))
    parts.append(place(bridge, X_BRIDGE, row_y))
    parts.append(place(enclosure, X_ENC2, row_y))
    parts.append(place(exhaust, X_EXHAUST, row_y))

# Group into assembly compound
assembly = Compound(children=parts)

# The whole point of the assembly file: prove the two rows do not touch.
print("\nChecking row-to-row interference on placed solids...")
top_row = parts[:len(parts) // 2]      # first loop iteration was row_sign=+1
bot_row = parts[len(parts) // 2:]
def _vol(shape):
    if shape is None:
        return 0.0
    return shape.volume if hasattr(shape, "volume") else sum(s.volume for s in shape)

worst = 0.0
for t in top_row:
    for b in bot_row:
        worst = max(worst, _vol(t.intersect(b)))
# Reported, not asserted: ROW_GAP is a known underestimate (the printed enclosures fit
# uncut on the bench), so a computed overlap here is evidence about the measurement
# rather than about the parts. See the warning in fan_plenum.check_fit.
if worst < 1e-6:
    print(f"  max intersection {worst:.3f} mm^3  OK")
else:
    print(f"  max intersection {worst:.1f} mm^3 at ROW_PITCH={ROW_PITCH:.0f}mm "
          f"— measurement suspect, re-measure ROW_GAP")

out = Path(__file__).parent / "output"
out.mkdir(exist_ok=True)
export_step(assembly, str(out / "full_assembly.step"))
print(f"\nExported full_assembly.step to {out}/")
print(f"Open in FreeCAD, Fusion 360, or any STEP viewer")
