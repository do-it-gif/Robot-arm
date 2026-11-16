# Robot-arm
A personal robot arm with 6 degree of freedom ment for learning and AI integrated studies.
# 6-DOF robot arm

A desktop six-axis arm (550 mm reach, 1 kg payload target) that I am designing and building joint by joint. Every joint gets a CAD model, printable STEP files and a MATLAB verification script that checks the design against its worst-case load.

Author: Abu Saimoom, mechanical engineering student, LUT University.

## Status

| Joint | Function | Reducer | Status |
|---|---|---|---|
| J1 | Base rotation (vertical axis) | 20:1 cycloidal, 6812 output bearing | **Built and running** (Nov 2026); calculation in this repo |
| J2 | Shoulder | Stacked 19.3:1 planetary + belt | In design |
| J3 | Elbow | 20:1 cycloidal module | In design |
| J4 | Forearm roll | 20:1 cycloidal module | In design |
| J5 | Wrist pitch | Compact 20:1 cycloidal, 6807 housing bearings | **Designed**; calculation shows open issues (below) |
| J6 | Tool rotation | To be decided | In design |

Control (planned): field-oriented control on brushless joints, 14-bit magnetic encoders, two STM32 controllers linked to ROS 2.

## Repository layout

```
j1/
  Assem1.SLDASM, *.SLDPRT     SolidWorks model of the base joint
  print/*.STEP                neutral files for printing or machining
  calculation/checks.m        MATLAB verification, sections 1-15 + pass/fail summary
j5/
  cad/                        SolidWorks model of the wrist joint
  print/*.STEP
  checks.m                    MATLAB verification
```

## Running the calculations

Open `checks.m` in MATLAB (base MATLAB, no toolboxes) and run it. Each script prints numbered sections (geometry, kinematics, load, torque, ring-pin forces, Hertz contact, output pins, shaft, bearings, stiffness, natural frequency, tip error, maximum allowable load) and ends with a pass/fail summary. Inputs are in section 0; SI units throughout.

## J1 base joint

Two 19-lobe printed discs phased 180 degrees, 20 steel ring pins (dia 5 x 20) on a 75 mm pin circle, 1 mm eccentricity. The housing rotates and is the output, carried on a 6812 bearing. 6001 eccentric bearings, 608 shaft-end bearings, M5 screws as output pins. NEMA17 stepper input.

**Load case.** The axis is vertical, so gravity puts no torque on the joint. The worst case is the arm stretched out horizontally: tilt moment on the output bearing plus the torque to accelerate the arm (modelled as a 3 kg rod with a 1 kg payload at 0.55 m, 90 deg/s reached in 0.3 s).

| Check | Result |
|---|---|
| Output torque needed | 3.5 Nm |
| Motor torque needed / pull-out safety factor | 0.22 Nm / 1.8 |
| Allowable continuous output torque | 5.8 Nm, limited by contact pressure on the printed disc |
| Disc contact pressure at motor stall (11.2 Nm) | 62 MPa against 45 MPa allowable: **fails at stall** |
| Output pin bending at stall (M5 thread root, cantilever) | 430 MPa, safety factor 1.5 against a required 2.0: **fails** |
| 6001 eccentric bearing | L10 about 80,000 h at stall torque; static safety 5.0 |
| 6812 output bearing under tilt moment | static safety about 12 |
| First torsional natural frequency | 7.9 Hz |
| Load-to-rotor inertia ratio | 189:1 against a target below 30: **fails** |

**What that means.** The joint carries its operating duty with margin. It does not survive full motor stall torque, so motor current has to be limited. The inertia ratio means moves must be ramped slowly.

**Next changes.** Replace the M5 screws with dia 5 shoulder bolts (bending safety factor rises to about 2.9). Measure backlash. Replace the input values still based on estimates (arm mass, bearing support spacing, motor pull-out torque) with measured ones.

## J5 wrist joint

Compact version of the same architecture for a NEMA17: 20 ring pins on a 48 mm pin circle, 0.75 mm eccentricity, two 4.75 mm discs, 6702 eccentric bearings, two 6807 housing bearings, 6001 shaft-end bearings, bare M5 output pins on a 30 mm circle.

**Load case.** Wrist pitch with the link horizontal: gravity torque acts on the joint axis (0.6 kg at 60 mm plus 1 kg payload at 120 mm).

| Check | Result |
|---|---|
| Gravity torque / total output torque needed | 1.53 Nm / 1.67 Nm |
| Motor pull-out safety factor | 4.3 |
| Allowable output torque | **1.45 Nm**, limited by disc contact pressure: below the 1.67 Nm needed |
| Payload rating that follows | 0.8 kg at 120 mm |
| Disc contact pressure at stall | 120 MPa against 45 MPa allowable: **fails** |
| 6702 eccentric bearing static safety at stall | 0.98 against a required 2.0: **fails** |
| Housing wall behind ring pins | 0.5 mm against a 1.0 mm minimum: **fails** |
| Bolt / insert clearance to shaft-end bearing seat | -2.5 mm (interference): **fails** |
| Output pin bending | safety factor 5.8 |
| First natural frequency | 90 Hz |

**What that means.** As drawn, the wrist joint is rated for 0.8 kg, not 1 kg, and has two packaging conflicts to resolve. Options under evaluation: smaller ring pins with a larger pin circle, a stiffer disc material, a larger eccentric bearing.

## Assumptions and limits of the calculations

- Printed PLA: E = 3 GPa, 45 MPa allowable contact pressure, 25 MPa allowable crush. These are short-term values; PLA creeps under sustained load.
- Bearing ratings are catalogue values derated by 0.7 for unbranded bearings.
- Contact and motor stiffness terms are first-order estimates (about +-50%).
- Arm link bending is not included yet.
- Calculations were written in November 2026 as verification of the built J1 and the designed J5.

## Related

Gear reducers, the planetary gearbox and the cycloidal profile design tool: see the `Gearbox` repository.
