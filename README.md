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

| Check | Result | Verdict |
|---|---|---|
| Output torque needed | 3.47 Nm | |
| Motor torque needed / pull-out safety factor | 0.218 Nm / 1.84 | ok |
| Allowable continuous output torque | 5.84 Nm, limited by disc contact | ok, 1.7x the need |
| Housing groove crush | 1.7 MPa, safety factor 14.6 | ok |
| 6001 eccentric bearing | L10 about 81,000 h at stall torque; static safety 5.0 | ok |
| 608 shaft-end bearings | static safety 12 | ok |
| 6812 output bearing under 13.5 Nm tilt moment | static safety 11.8 | ok |
| First torsional natural frequency | 7.9 Hz (limit 5 Hz) | ok |
| Disc contact pressure at motor stall (11.2 Nm) | 62 MPa against 45 MPa allowable | **fail** |
| Output pin bending at stall | 430 MPa, safety factor 1.49 against a required 2.0 | **fail** |
| Load-to-rotor inertia ratio | 189:1 against a guideline of 30:1 | **fail** |

### The three failed checks: cause and fix

| Failed check | Why it fails | Fix |
|---|---|---|
| Disc contact at stall | A steel pin on a printed PLA lobe is a narrow line contact. Full stall torque (11.2 Nm) is 3.2x what the joint needs, and the plastic surface is the weakest link. | Limit motor current so output torque stays below 5.8 Nm. That is still 1.7x the 3.47 Nm needed. Longer term: a tougher disc material. |
| Output pin bending | The M5 screws bend at the thread root (4.0 mm core) as cantilevers 15 mm long. | Replace with dia 5 shoulder bolts: safety factor about 2.9. Supporting the pins at both ends would give about 3.6. |
| Inertia ratio | The stretched arm has 0.605 kg m^2; through 20:1 that is still 189x the rotor inertia. This is a controllability guideline for open-loop steppers, not a strength limit. | Ramped acceleration (0.3 s ramp already assumed, pull-out margin 1.84), input shaping at the 7.9 Hz resonance, and encoder feedback on the joint. |

Still open: backlash is not measured (10 arcmin assumed, which is 1.6 mm at the tip), and some inputs are estimates (arm mass, bearing support spacing, motor pull-out torque).

## J5 wrist joint

Compact version of the same architecture for a NEMA17: 20 ring pins on a 48 mm pin circle, 0.75 mm eccentricity, two 4.75 mm discs, 6702 eccentric bearings, two 6807 housing bearings, 6001 shaft-end bearings, bare M5 output pins on a 30 mm circle.

**Load case.** Wrist pitch with the link horizontal: gravity torque acts on the joint axis (0.6 kg at 60 mm plus 1 kg payload at 120 mm).

| Check | Result | Verdict |
|---|---|---|
| Gravity torque / total output torque needed | 1.53 Nm / 1.67 Nm | |
| Motor pull-out safety factor | 4.28 | ok |
| Output pin bending at stall | 110 MPa, safety factor 5.8 | ok |
| 6807 housing bearings | static safety 15.4 | ok |
| First natural frequency | 90 Hz (limit 10 Hz) | ok |
| Inertia ratio | 5.2:1 | ok |
| Disc contact pressure at the 1 kg operating load | 48 MPa against 45 MPa allowable | **fail**, 7% over |
| Disc contact pressure at stall (10.4 Nm) | 120 MPa against 45 MPa | **fail** |
| Allowable output torque | 1.45 Nm against 1.67 Nm needed | **rated 0.8 kg, not 1 kg** |
| 6702 eccentric bearing static safety at stall | 0.98 against a required 2.0 | **fail** |
| Housing wall behind ring pins | 0.5 mm against a 1.0 mm minimum | **fail** |
| Bolt / insert clearance to shaft-end bearing seat | -2.5 mm | **fail** |

### The failed checks: cause and fix

| Failed check | Why it fails | Fix |
|---|---|---|
| Disc contact | The small 48 mm pin circle gives a tight lobe radius (effective contact radius about 1.1 mm), and the discs are only 4.75 mm thick. | Rated for 0.8 kg for now. About 15% more contact length (thicker discs with longer ring pins) brings the 1 kg operating case inside the limit. Motor current limited so stall torque is never reached. |
| 6702 static safety at stall | At full stall the eccentric bearing sees 419 N against a derated static rating of 413 N. | Same current limit: at the operating load it sees 67 N, a static safety above 6. |
| Housing wall | 54 mm housing over a 48 mm pin circle with dia 5 pins leaves 0.5 mm of printed wall. | Increase the housing diameter to 56 mm (1.5 mm wall), or use dia 4 ring pins (1.0 mm wall). |
| Bolt / insert vs bearing seat | The 7 mm heat-set inserts on the 30 mm bolt circle overlap radially with the 28 mm seat of the 6001 shaft-end bearing. | Use a smaller shaft-end bearing (a 608 clears by 0.5 mm, a 6702 by 1.0 mm), or separate insert and bearing seat axially. |

## Assumptions and limits of the calculations

- Printed PLA: E = 3 GPa, 45 MPa allowable contact pressure, 25 MPa allowable crush. These are short-term values; PLA creeps under sustained load.
- Bearing ratings are catalogue values derated by 0.7 for unbranded bearings.
- Contact and motor stiffness terms are first-order estimates (about +-50%).
- Arm link bending is not included yet.
- Calculations were written in November 2026 as verification of the built J1 and the designed J5.

## Related

Gear reducers, the planetary gearbox and the cycloidal profile design tool: see the `Gearbox` repository.
