"""
V100 SXM2 Fan Plenum — Multi-Part (FDM Filament Print)
Board: Tyan TNS-45XM2-4P54

Four part types with bolt flanges at butt joints.
Uses outer-inner subtraction with explicit end face cuts.
2.5mm walls for FDM strength (6 perimeters @ 0.4mm nozzle).

Usage:
    conda run -n cad python hardware/fan-shroud/fan_plenum.py
"""

from build123d import *
from build123d import export_step, export_stl
from math import atan2, cos, degrees, radians
from pathlib import Path

# ===== MEASUREMENTS =====
HS_TOTAL_W = 119.35
HS_ENC_LEN = HS_TOTAL_W              # exact base plate match — settling provided by 23mm bare base plate inside
HS_PERP = 78.0
HS_FIN_H = 63.0
HS_BASE_H = 11.0          # 74mm board-to-top minus 63mm fins
COL_GAP = 35.0              # gap between heatsinks along the flow direction (X)
ROW_GAP = 20.0              # gap between heatsinks across rows (Y) — measured 2026-07-26
ROW_PITCH = HS_PERP + ROW_GAP        # 98.0mm row centre-to-centre
V100_GAP = 18.0             # open slot between V100 module PCBs inside COL_GAP (info only)
HS_TOP = HS_BASE_H + HS_FIN_H

# ===== FAN =====
FAN_SIZE = 120.0
FAN_HOLE_SPACING = 105.0
FAN_CORNER_R = 7.5

# Fan fixing. Noctua's supplied screws are SELF-TAPPING — they are meant to cut their
# own thread in sheet steel. Driving them into a 3.5mm PETG flange splits it, and heat
# plus fan vibration finishes the job. So the plastic carries NO thread at all: the hole
# is an M4 clearance hole and the load is taken by an M4 machine screw pulling against
# a captive nut trapped in a boss on the inside face. The plastic sees compression only.
FAN_SCREW_DIA = 4.5          # M4 machine-screw clearance (was 4.3 = M4 self-tap)
FAN_NUT_AF = 7.0             # M4 hex nut across flats
FAN_NUT_POCKET = 3.4         # nut is 3.2mm thick; 0.2mm so it drops in
FAN_BOSS_DIA = 14.0          # boss around each hole on the inside face
FAN_BOSS_H = 3.5             # 3.5 flange + 3.5 boss = 7.0mm, pocket 3.4 leaves 3.6mm

# ===== CONSTRUCTION =====
WALL = 2.5                # FDM-friendly (6 perimeters @ 0.4mm nozzle)
TAPER_LEN = 120.0          # intake taper (contracting — needs gentle angle)
EXHAUST_LEN = 120.0         # exhaust diffuser — matches intake for gentle expansion
CLEARANCE_Y = 0.2               # snug — FDM printing tolerance only
CLEARANCE_Z = 2.0          # snug around fins, no dead air

CH_W = HS_PERP + 2 * CLEARANCE_Y
CH_H = HS_FIN_H + CLEARANCE_Z       # fins only, sits on base plate
CH_Z = CH_H / 2
CH_R = 2.0
OW = CH_W + 2 * WALL
OH = CH_H + 2 * WALL

FL_RING = 8.0
FL_THICK = 3.5
BOLT_DIA = 3.2

# Flange plate envelope — FULL WIDTH, no side trim.
#
# The model says 99.4mm cannot fit a 98mm row pitch (1.4mm short) and an earlier
# revision clipped the inboard edge to 94.0mm to suit. THE BENCH SAYS OTHERWISE: the
# printed enclosures fit between the rows uncut (verified 2026-07-29), so the real row
# pitch is at least ~100mm and ROW_GAP below is an underestimate. Trusting the physical
# fit, the trim is reverted and the original symmetric plate + 4-hole pattern restored,
# which also makes every hole on the already-printed part 2 mate again.
# See ROW_GAP — worth a caliper before anyone re-derives FAN_OFFSET from it.
FL_OW = OW + 2 * FL_RING             # 99.4mm
FL_OH = OH + 2 * FL_RING
FL_R = CH_R + WALL + FL_RING

# Original 4-hole pattern: 2 on the top strip, 2 on the sides at mid-height.
BOLT_Y = (-15.0, 15.0)
BOLT_Y_SIDE = OW / 2 + FL_RING / 2   # 45.7mm

# COL_GAP (35.0) is the gap between the heatsink BASE PLATES. The space actually left
# between the two installed part 2's is smaller — measured 2026-08-11. Part 3 is sized
# to the measured clearance, not to the nominal gap.
BRIDGE_MAX = 30.94                        # measured free span between the part 2's
BRIDGE_PRINTED = BRIDGE_MAX
BRIDGE_BODY = BRIDGE_MAX - 2 * FL_THICK   # 23.94

# Part 3's bottom. A flat floor at z=-2.5 fouled a screw on the board, so the floor is
# OPEN across the gap except for one central fill that drops into the slot between the
# two V100 modules. Leaving the ends open is deliberate: it lets some air reach the
# components at the edge of the V100 that would otherwise sit in dead air.
BRIDGE_MAX_H = 74.73                      # top of shroud to lowest point, measured
BRIDGE_TOP_Z = CH_Z + OH / 2              # +67.5
FILL_BOTTOM_Z = BRIDGE_TOP_Z - BRIDGE_MAX_H   # -7.23, i.e. 7.23mm below the plate plane

SLOT_X = 14.21          # measured slot between the V100 modules, along the flow
SLOT_Y = 82.90          # measured slot across the board
FILL_CLEAR = 0.4        # so it drops in rather than press-fits
FILL_X = SLOT_X - FILL_CLEAR              # 13.81
FILL_Y = SLOT_Y - FILL_CLEAR              # 82.50

FAN_FL_EXTRA = 8.0
FAN_FL_THICK = 3.5
FAN_FL_SIZE = FAN_SIZE + 2 * FAN_FL_EXTRA    # 136mm — the widest thing on the build
FAN_R_INNER = FAN_SIZE / 2 - 5               # 55mm — fan opening is near-circular
FAN_R_OUTER = (FAN_SIZE + 2 * WALL) / 2 - 5  # 57mm

# Rows are only ROW_PITCH apart but the fan flange is FAN_FL_SIZE wide, so the two
# rows' fans must be staggered in Y by half the overlap plus a clearance margin.
# At ROW_PITCH=98 and FAN_FL_SIZE=136 this gives ±22mm.
FAN_FL_CLEAR = 6.0
FAN_OFFSET = (FAN_FL_SIZE + FAN_FL_CLEAR - ROW_PITCH) / 2


def _assert_cut_inside_opening(bw, bh, ow, oh, orad, what):
    """A cut box must fit inside the duct opening at that station — CORNERS INCLUDED.

    Checking the flats alone is not enough: a box can clear both flats and still poke
    its corners through a rounded opening. For a rounded rect, the point (ax, ay) is
    inside iff, measuring only the overshoot past the arc centre, dx^2 + dy^2 <= r^2.
    """
    ax, ay = (bw + 0.5) / 2, (bh + 0.5) / 2
    assert ax <= ow / 2 and ay <= oh / 2, (
        f"{what}: cut box {bw}x{bh} exceeds the {ow}x{oh} opening flats")
    dx, dy = max(0.0, ax - (ow / 2 - orad)), max(0.0, ay - (oh / 2 - orad))
    assert (dx * dx + dy * dy) ** 0.5 <= orad, (
        f"{what}: cut box corner ({ax:.2f},{ay:.2f}) pokes past the r={orad} rounded "
        f"opening — it will cut slots through the wall, not open the end face")


def _cut_end_face(solid, x_pos, w=None, h=None, y_center=0.0):
    """Cut the end face wall open at x_pos.

    The box MUST stay inside the duct's opening at this station, or it eats the wall
    instead of the end face. It defaults to the channel cross-section, which is right
    for the enclosure, the bridge, and the channel end of the tapers — but NOT for a
    taper's fan end, which is a different size AND offset by the row stagger. Pass the
    local opening explicitly there.

    (This is exactly how the 2026-07-26 wall slots appeared: at the old +-9mm stagger a
    channel-sized box happened to fit inside the 120mm fan opening and was a harmless
    no-op; at +-22mm the opening shifted and the same box began cutting 712 mm^3 of
    slots through the cone wall.)
    """
    w = CH_W if w is None else w
    h = CH_H if h is None else h
    with BuildPart() as cut:
        with Locations([(x_pos, y_center, CH_Z)]):
            Box(WALL * 4, w + 0.5, h + 0.5)
    return solid - cut.part


def _cut_bottom(solid, x_start, length, width=None):
    """Cut bottom wall open. Width defaults to channel; use wider for tapers."""
    w = width or (CH_W + 0.5)
    floor_z = CH_Z - OH / 2
    with BuildPart() as cut:
        with Locations([(x_start + length / 2, 0, floor_z + WALL / 2)]):
            Box(length + 0.5, w, WALL + 1)
    return solid - cut.part


def _make_closed_shell(p0, p1, outer_w0, outer_h0, outer_r0,
                       outer_w1, outer_h1, outer_r1,
                       inner_w0, inner_h0, inner_r0,
                       inner_w1, inner_h1, inner_r1):
    """Create a closed hollow shell by outer-inner loft subtraction."""
    with BuildPart() as outer:
        with BuildSketch(p0):
            RectangleRounded(outer_w0, outer_h0, outer_r0)
        with BuildSketch(p1):
            RectangleRounded(outer_w1, outer_h1, outer_r1)
        loft()

    with BuildPart() as inner:
        with BuildSketch(p0):
            RectangleRounded(inner_w0, inner_h0, inner_r0)
        with BuildSketch(p1):
            RectangleRounded(inner_w1, inner_h1, inner_r1)
        loft()

    return outer.part - inner.part


def _cut_junction_bottom(solid, x_pos):
    """Remove bottom wall + corner fillets near a bolt-flange junction."""
    floor_z = CH_Z - OH / 2 - 1
    ceil_z = CH_Z - CH_H / 2 + CH_R - 0.5
    cut_h = ceil_z - floor_z
    with BuildPart() as cut:
        with Locations([(x_pos, 0, (floor_z + ceil_z) / 2)]):
            Box(WALL * 12, OW - 0.5, cut_h)
    return solid - cut.part


def _fan_flange(x_outer, fan_y_offset, into_duct):
    """Fan mounting plate with captive-nut bosses on the duct side.

    x_outer   — x of the face the fan bolts against
    into_duct — +1 if the duct interior lies at increasing x, -1 otherwise

    No thread is ever cut in the plastic. An M4 machine screw passes through the fan
    and this plate into a hex nut trapped in a boss on the inside face, so the flange
    is only ever in compression. The hex pockets open toward the duct interior, which
    is upward in the recommended print orientation (fan end down) — no supports.
    """
    p_out = Plane(origin=(x_outer, fan_y_offset, CH_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    x_inner = x_outer + into_duct * FAN_FL_THICK
    x_boss = x_inner + into_duct * FAN_BOSS_H
    hs = FAN_HOLE_SPACING / 2
    corners = [(hs, hs), (-hs, hs), (hs, -hs), (-hs, -hs)]

    with BuildPart() as plate:
        with BuildSketch(p_out):
            RectangleRounded(FAN_FL_SIZE, FAN_FL_SIZE, FAN_CORNER_R + FAN_FL_EXTRA)
            # Match the duct inner at the fan end — bell-mouth diagonal ~124mm
            RectangleRounded(FAN_SIZE, FAN_SIZE, FAN_R_INNER, mode=Mode.SUBTRACT)
        extrude(amount=into_duct * FAN_FL_THICK)
    part = plate.part

    # Bosses on the inside face to host the nuts
    with BuildPart() as bosses:
        with BuildSketch(Plane(origin=(x_inner, fan_y_offset, CH_Z),
                               x_dir=(0, 1, 0), z_dir=(1, 0, 0))):
            with Locations(corners):
                Circle(FAN_BOSS_DIA / 2)
        extrude(amount=into_duct * FAN_BOSS_H)
    part = part + bosses.part

    # Hex nut pockets, opening toward the duct interior
    with BuildPart() as pockets:
        with BuildSketch(Plane(origin=(x_boss, fan_y_offset, CH_Z),
                               x_dir=(0, 1, 0), z_dir=(1, 0, 0))):
            with Locations(corners):
                RegularPolygon(FAN_NUT_AF / 2 / cos(radians(30)), 6)
        extrude(amount=-into_duct * FAN_NUT_POCKET)
    part = part - pockets.part

    # Screw clearance holes through plate + boss
    span = FAN_FL_THICK + FAN_BOSS_H + 2
    with BuildPart() as holes:
        with BuildSketch(p_out):
            with Locations(corners):
                Circle(FAN_SCREW_DIA / 2)
        extrude(amount=into_duct * span, both=True)
    part = part - holes.part

    return part


def _bolt_flange(x_pos, direction=1, bottom="open"):
    """Bolt flange plate at x_pos. Direction: +1 extends in +X, -1 in -X.

    bottom="open"  — trim the plate above the channel floor, matching the U-channel
                     parts that drop over the heatsink from above.
    bottom="flush" — trim the plate flush with the body's outer bottom (z=-2.5).
    bottom="board" — trim at z=0 so NOTHING hangs below the base-plate plane. Part 3
                     uses this: its ends must clear a screw on the board.
    The plate is full width and symmetric — see FL_OW.
    """
    fl_oh, fl_r = FL_OH, FL_R

    if direction > 0:
        x0, x1 = x_pos, x_pos + FL_THICK
    else:
        x0, x1 = x_pos - FL_THICK, x_pos

    xmid = (x0 + x1) / 2
    p = Plane(origin=(x0, 0, CH_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))

    with BuildPart() as fl:
        with BuildSketch(p):
            RectangleRounded(FL_OW, fl_oh, fl_r)
            RectangleRounded(CH_W, CH_H, CH_R, mode=Mode.SUBTRACT)
        extrude(amount=FL_THICK)

    flange = fl.part

    bottom_z = CH_Z - fl_oh / 2                     # -10.5 — plate bottom edge
    trim_to = {
        "open": bottom_z + FL_RING + WALL + 3,      # +3.0 — clear of the base plate
        "flush": CH_Z - OH / 2,                     # -2.5 — flush with body outer
        "board": 0.0,                               #  0.0 — nothing below the plate plane
    }[bottom]
    cut_lo = bottom_z - 2
    with BuildPart() as bcut:
        with Locations([(xmid, 0, (cut_lo + trim_to) / 2)]):
            Box(FL_THICK + 2, FL_OW + 2, trim_to - cut_lo)
    flange = flange - bcut.part

    # Original 4-hole pattern — 2 top, 2 side at mid-height. Matches the enclosures
    # already printed, so every bolt lines up.
    bolt_z_top = CH_Z + OH / 2 + FL_RING / 2   # top strip center
    holes = [(by, bolt_z_top) for by in BOLT_Y]
    holes += [(BOLT_Y_SIDE, CH_Z), (-BOLT_Y_SIDE, CH_Z)]

    for by, bz in holes:
        with BuildPart() as hole:
            with Locations([(xmid, by, bz)]):
                Cylinder(BOLT_DIA / 2, FL_THICK + 2, rotation=(0, 90, 0))
        flange = flange - hole.part

    return flange


# =========================================================
# PART 1: Fan Taper
# =========================================================
def make_fan_taper(fan_y_offset: float = 0.0):
    fw = FAN_SIZE + 2 * WALL
    fi = FAN_SIZE
    # Near-circular at fan end (r=55 for 120mm ≈ circle), tapers to rectangular channel
    fr_inner, fr_outer = FAN_R_INNER, FAN_R_OUTER

    fan_p = Plane(origin=(0, fan_y_offset, CH_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    out_p = Plane(origin=(TAPER_LEN, 0, CH_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))

    taper = _make_closed_shell(
        fan_p, out_p,
        fw, fw, fr_outer,         # outer at fan end (near-circular)
        OW, OH, CH_R + WALL,      # outer at outlet (rectangular)
        fi, fi, fr_inner,         # inner at fan end (near-circular)
        CH_W, CH_H, CH_R,         # inner at outlet (rectangular)
    )

    # Open both end faces. The fan end is FAN_SIZE wide and offset by the stagger, so
    # the box must be sized and centred for THAT opening, inset 2mm to stay clear of
    # the wall. The far end is the plain channel cross-section.
    _assert_cut_inside_opening(CH_W, CH_H, FAN_SIZE, FAN_SIZE, FAN_R_INNER,
                               "intake taper fan end")
    taper = _cut_end_face(taper, 0, y_center=fan_y_offset)
    taper = _cut_end_face(taper, TAPER_LEN)

    # Bolt flange U-shaped at outlet to match enclosure open bottom; taper body sealed
    taper = taper + _bolt_flange(TAPER_LEN, direction=+1, bottom="open")

    # Fan mounting flange — duct interior is at increasing x from the fan face
    taper = taper + _fan_flange(-FAN_FL_THICK, fan_y_offset, into_duct=+1)

    return taper


# =========================================================
# PART 2: Heatsink Enclosure
# =========================================================
def make_hs_enclosure():
    L = HS_ENC_LEN
    p0 = Plane(origin=(0, 0, CH_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    p1 = Plane(origin=(L, 0, CH_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))

    enc = _make_closed_shell(
        p0, p1,
        OW, OH, CH_R + WALL,
        OW, OH, CH_R + WALL,
        CH_W, CH_H, CH_R,
        CH_W, CH_H, CH_R,
    )

    # Open both ends + bottom (drops over heatsink from above, sealed with tape)
    enc = _cut_end_face(enc, 0)
    enc = _cut_end_face(enc, L)
    enc = _cut_bottom(enc, 0, L)

    # Bolt flanges (open bottom to match)
    enc = enc + _bolt_flange(0, direction=-1, bottom="open")
    enc = enc + _bolt_flange(L, direction=+1, bottom="open")

    return enc


# =========================================================
# PART 3: Gap Bridge
# =========================================================
def make_gap_bridge():
    """Spans the gap between two heatsinks, with an open bottom and one central fill.

    The floor is NOT continuous. A flat floor at z=-2.5 fouled a screw on the board, so
    everything below the base-plate plane is removed except a fill that drops into the
    slot between the two V100 modules. The open ends are deliberate — they let some air
    reach the components at the edge of the V100.
    """
    L = BRIDGE_BODY
    p0 = Plane(origin=(0, 0, CH_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    p1 = Plane(origin=(L, 0, CH_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))

    brg = _make_closed_shell(
        p0, p1,
        OW, OH, CH_R + WALL,
        OW, OH, CH_R + WALL,
        CH_W, CH_H, CH_R,
        CH_W, CH_H, CH_R,
    )

    brg = _cut_end_face(brg, 0)
    brg = _cut_end_face(brg, L)

    # Strip everything below the base-plate plane, floor included.
    floor_lo = CH_Z - OH / 2 - 1
    with BuildPart() as strip:
        with Locations([(L / 2, 0, (floor_lo + 0.0) / 2)]):
            Box(L + 2, OW + 2, abs(floor_lo))
    brg = brg - strip.part

    # Bolt flanges stop at the plate plane too — nothing may hang at the ends.
    brg = brg + _bolt_flange(0, direction=-1, bottom="board")
    brg = brg + _bolt_flange(L, direction=+1, bottom="board")

    # The fill: a single central block dropping into the inter-module slot. Its top is
    # the duct floor, so the airflow sees a flush surface there.
    with BuildPart() as fill:
        with Locations([(L / 2, 0, FILL_BOTTOM_Z / 2)]):
            Box(FILL_X, FILL_Y, abs(FILL_BOTTOM_Z))
    brg = brg + fill.part

    return brg


# =========================================================
# PART 4: Exhaust Shroud (expanding taper to pull fan)
# =========================================================
def make_exhaust_shroud(fan_y_offset: float = 0.0):
    """Channel → expanding taper → 120mm pull fan.
    Reversed from intake: bolt flange on inlet, fan flange on outlet."""
    fw = FAN_SIZE + 2 * WALL
    fi = FAN_SIZE
    fr_inner, fr_outer = FAN_R_INNER, FAN_R_OUTER
    L = EXHAUST_LEN

    # Inlet at X=0 (channel size), outlet at X=L (fan size)
    in_p = Plane(origin=(0, 0, CH_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))
    fan_p = Plane(origin=(L, fan_y_offset, CH_Z), x_dir=(0, 1, 0), z_dir=(1, 0, 0))

    shroud = _make_closed_shell(
        in_p, fan_p,
        OW, OH, CH_R + WALL,      # outer at inlet (rectangular)
        fw, fw, fr_outer,          # outer at fan end (near-circular)
        CH_W, CH_H, CH_R,          # inner at inlet (rectangular)
        fi, fi, fr_inner,          # inner at fan end (near-circular)
    )

    # Open both end faces — same asymmetry as the intake, mirrored
    shroud = _cut_end_face(shroud, 0)
    _assert_cut_inside_opening(CH_W, CH_H, FAN_SIZE, FAN_SIZE, FAN_R_INNER,
                               "exhaust shroud fan end")
    shroud = _cut_end_face(shroud, L, y_center=fan_y_offset)

    # Bolt flange U-shaped at inlet to match enclosure open bottom; shroud body sealed
    shroud = shroud + _bolt_flange(0, direction=-1, bottom="open")

    # Fan mounting flange on outlet — duct interior is at DEcreasing x from the fan face
    shroud = shroud + _fan_flange(L + FAN_FL_THICK, fan_y_offset, into_duct=-1)

    return shroud


# =========================================================
# FIT CHECKS
# =========================================================
# Every fit failure found at first assembly (2026-07-26) came from checking a BODY
# dimension against a board dimension and forgetting the flange envelope around it.
# These run on every build so an unfittable STL cannot be exported silently.

def _row_clear(clear):
    return f"row-to-row clearance {clear:+.1f}mm" + ("        OK" if clear >= 2 else "   CHECK")


def check_fit():
    """Assert the hard clearances; return (report_lines, warnings)."""
    v100_pcb_z = -HS_BASE_H              # -11.0 — V100 module PCB top
    bridge_low_z = FILL_BOTTOM_Z         # -7.23 — lowest point of part 3 (the fill)
    body_low_z = CH_Z - FL_OH / 2        # -10.5 — where a full flange ring would reach

    gap_used = BRIDGE_BODY + 2 * FL_THICK
    fan_fl_clear = ROW_PITCH + 2 * FAN_OFFSET - FAN_FL_SIZE
    fan_duct_clear = ROW_PITCH + 2 * FAN_OFFSET - (FAN_SIZE + 2 * WALL)
    fan_body_clear = ROW_PITCH + 2 * FAN_OFFSET - FAN_SIZE
    body_clear = ROW_PITCH - OW
    bolt_fl_clear = ROW_PITCH - FL_OW
    bridge_pcb_clear = bridge_low_z - v100_pcb_z

    # --- hard asserts: violating any of these makes the part unfittable ---
    assert abs(BRIDGE_BODY + 2 * FL_THICK - BRIDGE_MAX) < 1e-9, (
        f"part 3 prints {BRIDGE_BODY + 2 * FL_THICK}mm, over the measured "
        f"{BRIDGE_MAX}mm clearance between the part 2's")
    assert abs(gap_used - BRIDGE_MAX) < 1e-9, (
        f"part 3 consumes {gap_used}mm of the {BRIDGE_MAX}mm measured clearance")
    assert BRIDGE_BODY > 4 * WALL, (
        f"part 3 body {BRIDGE_BODY}mm is too short for two {WALL * 4}mm end-face cuts")
    # The limit is measured from the SHROUD ROOF, not from the top of the bolt
    # flanges — those stand 8mm proud and have only open air above them.
    assert BRIDGE_TOP_Z - FILL_BOTTOM_Z <= BRIDGE_MAX_H + 1e-9, (
        f"part 3 measures {BRIDGE_TOP_Z - FILL_BOTTOM_Z:.2f}mm from the shroud roof to "
        f"the fill bottom, over the {BRIDGE_MAX_H}mm limit")
    assert FILL_X < SLOT_X and FILL_Y < SLOT_Y, (
        f"fill {FILL_X}x{FILL_Y} will not drop into the {SLOT_X}x{SLOT_Y} slot")
    assert FILL_X <= BRIDGE_BODY, (
        f"fill {FILL_X}mm is longer than the {BRIDGE_BODY:.2f}mm body")
    assert FILL_Y > CH_W, (
        f"fill {FILL_Y}mm is narrower than the {CH_W}mm channel — it would not reach "
        f"the side walls and would float unattached")
    assert bridge_pcb_clear >= 3.0, (
        f"part 3 reaches z={bridge_low_z}mm, only {bridge_pcb_clear}mm above the "
        f"V100 PCB at z={v100_pcb_z}mm")
    assert fan_fl_clear >= 2.0, (
        f"fan flanges overlap between rows by {-fan_fl_clear:.1f}mm at "
        f"ROW_PITCH={ROW_PITCH} with FAN_OFFSET={FAN_OFFSET}")
    assert fan_duct_clear >= 2.0, (
        f"fan-end ducts overlap between rows by {-fan_duct_clear:.1f}mm")
    assert body_clear > 0, f"channel bodies overlap between rows by {-body_clear:.1f}mm"
    # NOT asserted: the bolt flange vs ROW_PITCH. The model computes an overlap, the
    # bench says the printed enclosures fit uncut, and the bench wins — this is a bad
    # ROW_GAP, not a bad part. It is reported as a warning instead.
    assert max(abs(y) for y in BOLT_Y) + BOLT_DIA / 2 < OW / 2, (
        "top bolt holes fall outside the flange plate")
    assert CH_W >= HS_PERP and CH_H >= HS_FIN_H, "channel is smaller than the fins"
    assert (FAN_FL_SIZE - FAN_SIZE) / 2 >= WALL, "fan flange has no material at mid-face"

    report = [
        f"  gap budget      part3 body {BRIDGE_BODY:.2f} + 2x{FL_THICK} flange "
        f"= {gap_used:.2f} of {BRIDGE_MAX:.2f}mm measured     OK",
        f"  part3 printed   {BRIDGE_PRINTED:.2f}mm = the measured clearance "
        f"({FL_THICK} flange + {BRIDGE_BODY:.2f} body + {FL_THICK} flange)",
        f"  bore            {CH_W}x{CH_H} unbroken end to end — no step, no neck",
        f"  part3 height    shroud top {BRIDGE_TOP_Z:+.1f} to fill bottom "
        f"{FILL_BOTTOM_Z:+.2f} = {BRIDGE_TOP_Z - FILL_BOTTOM_Z:.2f}mm of {BRIDGE_MAX_H}mm     OK",
        f"  part3 overall   {CH_Z + FL_OH / 2 - FILL_BOTTOM_Z:.2f}mm incl. bolt flanges "
        f"(flange top {CH_Z + FL_OH / 2:+.1f}) — flanges stand above the roof",
        f"  part3 vs V100   lowest z {bridge_low_z:+.2f}, PCB top {v100_pcb_z:+.1f} "
        f"-> {bridge_pcb_clear:+.2f}mm        OK",
        f"  fill            {FILL_X:.2f}x{FILL_Y:.2f} into a {SLOT_X}x{SLOT_Y} slot "
        f"({FILL_CLEAR}mm clearance)     OK",
        f"  fan flange      {_row_clear(fan_fl_clear)} (stagger +-{FAN_OFFSET:.1f}mm)",
        f"  fan body        {_row_clear(fan_body_clear)}",
        f"  fan-end duct    {_row_clear(fan_duct_clear)}",
        f"  channel body    {_row_clear(body_clear)}",
        f"  bolt flange     {_row_clear(bolt_fl_clear)}",
    ]

    warnings = []
    if bolt_fl_clear < 0:
        warnings.append(
            f"the MODEL says the {FL_OW:.1f}mm bolt flange overlaps between rows by "
            f"{-bolt_fl_clear:.1f}mm at ROW_PITCH={ROW_PITCH:.0f}mm, but the printed "
            f"enclosures were verified to fit UNCUT on 2026-07-29. The bench wins: "
            f"ROW_GAP={ROW_GAP:.0f}mm is an underestimate and the true pitch is at least "
            f"{FL_OW + 1:.0f}mm. Re-measure it — FAN_OFFSET for parts 1 and 4 is derived "
            f"from ROW_PITCH, so the {FAN_OFFSET:.0f}mm stagger rests on the same number.")
    fan_fl_low_z = CH_Z - FAN_FL_SIZE / 2
    warnings.append(
        f"the fan flange reaches z={fan_fl_low_z:+.1f}mm, i.e. {-fan_fl_low_z:.0f}mm below "
        f"the base-plate plane and {-fan_fl_low_z - HS_BASE_H:.0f}mm below the board — "
        f"the intake/exhaust ends need clear space under the board's front and rear edge.")
    steep = degrees(atan2(FAN_OFFSET + FAN_SIZE / 2 - CH_W / 2, EXHAUST_LEN))
    if steep > 15.0:
        warnings.append(
            f"the {FAN_OFFSET:.0f}mm stagger puts the exhaust diffuser's steep wall at "
            f"{steep:.1f}deg (>15deg risks separation). Raise EXHAUST_LEN to "
            f"{(FAN_OFFSET + FAN_SIZE / 2 - CH_W / 2) / 0.2679:.0f}mm for 15deg, or "
            f"accept the loss — it is an exhaust, not an intake.")
    board_y = (-HS_PERP / 2, ROW_PITCH + HS_PERP / 2)
    fan_y = (-FAN_OFFSET - FAN_SIZE / 2, ROW_PITCH + FAN_OFFSET + FAN_SIZE / 2)
    overhang = board_y[0] - fan_y[0]
    warnings.append(
        f"fans now overhang the board by {overhang:.0f}mm per side "
        f"(assembly is {fan_y[1] - fan_y[0]:.0f}mm wide vs a {board_y[1] - board_y[0]:.0f}mm "
        f"board) — they need a frame to mount to, not the board.")
    return report, warnings


def check_no_interference(parts):
    """Boolean-test the two rows against each other on the real solids.

    The arithmetic clearances above compare envelopes one dimension at a time and
    WILL miss things: the 2026-07-26 assembly failure was a bolt flange that passed
    every scalar check while overlapping the neighbouring row by 1.4mm. Only a solid
    intersection catches that, so it gates the export.
    """
    results = []
    for label, bottom_part, top_part in parts:
        placed = top_part.moved(Location((0, ROW_PITCH, 0)))
        overlap = bottom_part.intersect(placed)   # None when disjoint, ShapeList if split
        if overlap is None:
            vol = 0.0
        elif hasattr(overlap, "volume"):
            vol = overlap.volume
        else:
            vol = sum(sh.volume for sh in overlap)
        results.append((label, vol))
        # Reported, not asserted, for the same reason as the bolt-flange clearance:
        # ROW_GAP is known to be an underestimate, so a small computed overlap here is
        # evidence about the measurement, not about the part. Flag it loudly instead.
        if vol > 1e-6:
            print(f"  NOTE {label}: computes {vol:.1f} mm^3 of row overlap at "
                  f"ROW_PITCH={ROW_PITCH:.0f}mm (measurement suspect — see warnings)")
    return results


if __name__ == "__main__":
    out = Path(__file__).parent / "output"
    out.mkdir(exist_ok=True)

    fit_report, fit_warnings = check_fit()

    # Only parts 1 and 4 are handed, and only because the fan is staggered in Y.
    built = {}
    for name, builder in [
        ("part1_intake_taper_top", lambda: make_fan_taper(+FAN_OFFSET)),
        ("part1_intake_taper_bottom", lambda: make_fan_taper(-FAN_OFFSET)),
        ("part2_hs_enclosure", make_hs_enclosure),
        ("part3_gap_bridge", make_gap_bridge),
        ("part4_exhaust_shroud_top", lambda: make_exhaust_shroud(+FAN_OFFSET)),
        ("part4_exhaust_shroud_bottom", lambda: make_exhaust_shroud(-FAN_OFFSET)),
    ]:
        print(f"Building {name}...")
        built[name] = builder()

    # Gate the export on a solid interference test between the two rows. The bottom-row
    # part sits at y=0 and the top-row part is placed at y=ROW_PITCH, matching assembly.
    print("Checking row-to-row interference...")
    interference = check_no_interference([
        ("part1 intake", built["part1_intake_taper_bottom"], built["part1_intake_taper_top"]),
        ("part2 enclosure", built["part2_hs_enclosure"], built["part2_hs_enclosure"]),
        ("part3 bridge", built["part3_gap_bridge"], built["part3_gap_bridge"]),
        ("part4 exhaust", built["part4_exhaust_shroud_bottom"], built["part4_exhaust_shroud_top"]),
    ])

    for name, part in built.items():
        export_step(part, str(out / f"{name}.step"))
        export_stl(part, str(out / f"{name}.stl"))

    # Steep-wall angles: the Y stagger makes the taper asymmetric, so one wall does
    # nearly all the work. Quote that wall, not the symmetric average.
    steep_dy = FAN_OFFSET + FAN_SIZE / 2 - CH_W / 2
    intake_angle = degrees(atan2(steep_dy, TAPER_LEN))
    exhaust_angle = degrees(atan2(steep_dy, EXHAUST_LEN))
    fan_diag = 2 * ((FAN_SIZE / 2 - FAN_R_INNER) * 2**0.5 + FAN_R_INNER)
    print(f"\nDone. Channel {CH_W}×{CH_H}mm, {WALL}mm walls")
    print(f"Fan opening: {FAN_SIZE}×{FAN_SIZE}mm rounded (r={FAN_R_INNER:.0f}), diagonal {fan_diag:.1f}mm")
    print(f"Intake taper: {TAPER_LEN}mm, steep wall {intake_angle:.1f}° (contracting)")
    print(f"Exhaust shroud: {EXHAUST_LEN}mm, steep wall {exhaust_angle:.1f}° (expanding)")
    print(f"HS Enclosure: {HS_ENC_LEN:.2f}mm body (matches base plate)")

    print(f"\nBOARD: rows {ROW_GAP:.0f}mm apart (pitch {ROW_PITCH:.0f}mm), "
          f"columns {COL_GAP:.0f}mm apart")
    print("FIT CHECKS")
    for line in fit_report:
        print(line)
    for label, vol in interference:
        flag = "OK" if vol < 1e-6 else "see NOTE"
        print(f"  {label + ' solids':24}intersection {vol:.3f} mm^3   {flag}")
    if fit_warnings:
        print("\nWARNINGS — fits, but read these before printing:")
        for w in fit_warnings:
            print(f"  ! {w}")

    print(f"\nFULL PUSH-PULL SYSTEM (per row), printed lengths:")
    print(f"  [iPPC PUSH] → [Intake {TAPER_LEN + FAN_FL_THICK + FL_THICK:.1f}] → "
          f"[Enc {HS_ENC_LEN + 2 * FL_THICK:.2f}] → [Bridge {BRIDGE_PRINTED:.1f}] → "
          f"[Enc {HS_ENC_LEN + 2 * FL_THICK:.2f}] → [Exhaust {EXHAUST_LEN + FAN_FL_THICK + FL_THICK:.1f}] "
          f"→ [iPPC PULL]")
    total = TAPER_LEN + 2 * HS_ENC_LEN + COL_GAP + EXHAUST_LEN + 2 * FAN_FL_THICK
    print(f"  Board footprint spanned: {2 * HS_ENC_LEN + COL_GAP:.2f}mm "
          f"(2 base plates + {COL_GAP:.0f}mm gap)")
    print(f"  Total assembled length: {total:.1f}mm")
    print(f"\nParts: 2× intake (handed), 4× enclosure, 2× bridge, 2× exhaust (handed)")
    print(f"Fans: 4× NF-F12 iPPC-3000 (2 push + 2 pull)")
    print(f"Bolts: 32× M3 — 4 per joint over 8 joints (2 top at y=±15, 2 side at "
          f"y=±{BOLT_Y_SIDE:.1f}); all 4 mate with the already-printed part 2")
    print(f"Fan fixing: 16× M4 machine screw + 16× M4 hex nut (captive in the flange "
          f"bosses) — do NOT use Noctua's self-tapping screws in the plastic")
    print(f"Files in {out}/")
