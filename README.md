# V100 SXM2 fan shroud

A 3D-printable push-pull air duct for a 4-GPU NVIDIA V100 SXM2 NVLink carrier
board (Tyan TNS-4SXM2-4P54 type, DGX-1-style copper heatsinks), cooled by
ordinary 120 mm fans instead of a server's 16,000 RPM blowers.

Write-up with photos and an interactive 3D view:
<https://riichard.com/projects/gpu-cluster/>

## How it works

The heatsink fins run horizontally, so air enters from the side. Each row of
two GPUs gets one sealed duct, with a 120 mm fan pushing at the intake and a
second pulling at the exhaust:

```
[fan PUSH] → [intake taper] → [enclosure] → [gap bridge] → [enclosure] → [exhaust] → [fan PULL]
```

- The duct is sealed: between the two GPUs the air is above ambient pressure,
  so side openings would leak hot air out rather than draw fresh air in.
- The two rows sit 98 mm apart, closer than a 120 mm fan, so the fans are
  staggered ±22 mm and the intake and exhaust tapers are asymmetric.
- The gap bridge has an open floor with a central fill that plugs the slot
  between the two V100 modules.

## Parts (per 4-GPU board)

| File | Qty | Notes |
|---|---:|---|
| `part1_intake_taper_top` | 1 | fan face 120×120 → 80×65 channel |
| `part1_intake_taper_bottom` | 1 | mirrored stagger for the other row |
| `part2_hs_enclosure` | 4 | U-channel, drops over each heatsink |
| `part3_gap_bridge` | 2 | 30.94 mm, sized to the measured clearance |
| `part4_exhaust_shroud_top` | 1 | 80×65 channel → 120×120 fan face |
| `part4_exhaust_shroud_bottom` | 1 | |

`stl/` is for slicing; `step/` is for CAD, including `full_assembly.step`
with both rows in place. `shroud.glb` is a web-viewable copy of the assembly.

**`_top` and `_bottom` are not interchangeable.** A `_top` taper on the bottom
row puts the full-width flange inboard and it fouls the other row.

### Hardware

| Item | Qty |
|---|---:|
| Noctua NF-F12 iPPC-3000 PWM (or any 120 mm high static pressure fan) | 4 |
| M3 × 10 bolts + nuts (flange joints) | 32 |
| M4 × 35 machine screws + hex nuts + washers (fans) | 16 each |
| Aluminium tape for the joint seams | 1 roll |
| Foam weatherstrip for the enclosure bottom edges | 1 roll |
| PETG filament | ~250 g |

Don't use the self-tapping screws that come with the fans: they split a 3.5 mm
PETG flange. The fan flanges have hex pockets for M4 nuts, so the plastic only
takes compression. M4 heat-set inserts fit the bosses too.

## Printing

- **PETG.** PLA softens near the hot exhaust.
- FDM, 0.4 mm nozzle, 2.5 mm walls (6 perimeters).
- Tapers and exhausts: fan end down. Enclosures: upside-down, open side up.
  Bridges: upside-down or on their side. None of them need internal supports.
- Seal every bolted seam with aluminium tape, and the enclosure bottom edges
  against the heatsink base plate with foam strip.

## Measure your board first

All dimensions come from one board measured with calipers: heatsink base
plate 119.35 × 78 mm, fins 63 mm tall, 35 mm between GPU columns and 20 mm
between rows. Check yours before printing. The parametric source is in `src/`:

```bash
pip install build123d
python src/fan_plenum.py   # all parts → src/output/
python src/assembly.py     # full_assembly.step
```

The constants at the top of `fan_plenum.py` are the measured board dimensions.
The build checks that the two rows don't intersect before it exports anything.

## License

Copyright Richard Kraaijenhagen, Tulp Fusion Foundation.

Licensed under the CERN Open Hardware Licence Version 2 – Permissive
(CERN-OHL-P-2.0). See `LICENSE`.
