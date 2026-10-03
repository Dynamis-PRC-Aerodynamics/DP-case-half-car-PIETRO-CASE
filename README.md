# `DP-half-car-PIETRO-CASE`

Reference OpenFOAM case for the aerodynamic development of the DP18, as it runs on Pietro, the team workstation. Copy this folder, drop the geometry in, set the ride height and run: every DP18 development run on Pietro starts from here.

The case simulates **half a car in a straight line**, with a symmetry plane on the car centreline. What changes from one run to the next is the geometry, the ride height (front and rear, so the car can be pitched for braking and squat) and the front wing setting. Roll and yaw are not part of this case: cornering has its own cases.

It is the heavy counterpart of `DP-case-half-car-personal-case`, which is the same car on a mesh small enough for a personal computer. Patches, boundary conditions, force coefficients and scripts are the same in the two cases; the mesh is not:

| | Personal case | This case |
|---|---|---|
| Machine | personal PC, 12 cores | Pietro, 56 cores |
| Surface refinement | level 8 (level 9 on `FW`) | level 9 |
| Refinement around the car | one distance band per patch | cascades of three to six bands |
| Refinement boxes | 6 | 14 |
| Layer addition | default mesh shrinker | `displacementMotionSolver` |
| Iterations | 600 | 800 |
| Front-wheel steer in the pre-processing | no | yes, off at `steer = 0` |
| Run queue (`runQueue`) | no | yes |


---

## Quick start

What the machine needs:

- OpenFOAM v2512 in `/usr/lib/openfoam/openfoam2512`. The scripts source it themselves.
- `python3` with `numpy`, for the two scripts in `preProcessor/` and `postProcessor/`.
- The team library `forceCoeffsModified.so`, compiled against the same OpenFOAM version. Without it no coefficient is written: see [Reading the coefficients](#reading-the-coefficients).
- aeroTools, only for the plots and the report after the run.

Then:

1. Copy the whole folder and rename the copy after the run.
2. Put the 19 `.obj` files in `constant/triSurface_0deg/`, exactly as they come out of CAD.
3. Set `FRH` and `RRH` in `preProcessor/setup.txt`.
4. Run `./runMesh`, then check `log_mesh/05_snappyHexMesh` and `log_mesh/06_checkMesh`.
5. Run `./runSolve`.

`./runAll` does steps 4 and 5 in one go, without the stop to look at the mesh.

---

## The physical model

| | |
|---|---|
| Solver | `simpleFoam`, steady incompressible RANS, SIMPLEC (`consistent yes`) |
| Turbulence | k-omega SST with wall functions |
| Fluid | air, `nu = 1.5e-05` m²/s, `rho = 1.225` kg/m³ for the forces |
| Freestream | 16 m/s along `+x` |
| Domain | `x` from −35 to 65 m, `y` from 0 to 50 m, `z` from 0 to 50 m |
| Ground | moving wall at the freestream velocity |
| Wheels | `rotatingWallVelocity`, one rotation centre per axle |
| Tyre plinths | `slip` |
| `y = 0` | `symmetry` |
| Side and top | `symmetryPlane` |
| Inlet | fixed velocity, `k` and `omega` |
| Outlet | fixed pressure, `inletOutlet` on velocity |
| Radiator and battery pack | porous media, each followed by fan disks |
| Initial field | `potentialFoam` |

Velocity is discretised with `linearUpwindV`, `k` and `omega` with `upwind`. The run is 800 iterations long and writes every 200, keeping only the last write (`purgeWrite 1`). `iterations` and `writeStep` are in `initialConditions`.

---

## The two reference frames

CAD has its origin on the rear axle, `x` towards the nose and `y` towards the left. OpenFOAM has its origin on the front axle, `x` towards the rear and `y` towards the right. `z` points up in both.

The conversion is a 180 degree rotation about `z` plus a wheelbase translation in `x`:

```
x_aero = -x_cad + 1.53      y_aero = -y_cad      z_aero = z_cad
```

So the front axle sits at `x = 0`, the rear axle at `x = 1.53`, and the half that is kept is `y > 0`, the right-hand side of the car.

The pre-processing does this conversion on every `.obj`, before anything else. Put the geometries in `constant/triSurface_0deg/` **exactly as they come out of CAD**: transform them by hand and they get transformed twice, leaving the car 3.06 m away from the origin.

---

## The geometries expected in `triSurface_0deg/`

Nineteen files, case-sensitive:

`Diffuser` · `Fan_Battery` · `Fan_Radiators` · `FW` · `Monocoque` · `Monocoque_Duct` · `Monocoque_Kick` · `Porous_Battery` · `Porous_Radiators` · `Radiator_Ducts` · `RW` · `RW_Upper` · `Susps` · `T_Tray` · `Tyre_Plinths` · `UT` · `Uprights` · `Wheels` · `Winglets`

They are not versioned: the repository holds an empty `constant/triSurface_0deg/`.

Two of the files hold more than one body, and the pre-processing splits them, so `constant/triSurface/` ends up with 21 files:

| File from CAD | Becomes | Why |
|---|---|---|
| `Wheels.obj` | `Wheels_Front.obj` + `Wheels_Rear.obj` | `rotatingWallVelocity` takes one rotation centre per patch, and the two axles have different ones |
| `Fan_Battery.obj` | `Fan_Battery_1.obj` + `Fan_Battery_2.obj` | each disk needs its own `cellZone` and its own fan source |

The wheels are split on the plane `x = 0.765`, half the wheelbase.

The fans are split by connected components, one output file per disk. The CAD exports the whole car, so the battery pack hands over four fans; the symmetry plane cuts two of them out of the domain and the pre-processing drops them, saying so on screen. A disk entirely at `y < 0` would produce an empty `cellZone` and an `fvOptions` source that applies nothing without complaining. The two survivors are named by increasing `y` of their centroid, so a new CAD export cannot swap which physical fan is `Fan_Battery_1`.

### The monocoque is three patches

In CAD the monocoque is a single part. It is exported as three files to control the layer extrusion separately on each:

| Patch | What it is | Layers |
|---|---|---|
| `DP_Monocoque` | the rest of the monocoque | `$Monocoque_nLayers` |
| `DP_Monocoque_Kick` | the central zone of the suction side, facing the underbody | `$Monocoque_Kick_nLayers` |
| `DP_Monocoque_Duct` | internal ducts feeding air from the sidepods into the battery pack | 0 |

All three are needed to cover the monocoque. The duct gets no layers because extrusion inside it fails.

---

## Patches and assemblies

Each wall patch is named after its file with a `DP_` prefix: `UT.obj` becomes `DP_UT`. That gives **16 wall patches**.

`Porous_Battery`, `Porous_Radiators`, `Fan_Battery_1`, `Fan_Battery_2` and `Fan_Radiators` carry no `DP_` prefix on purpose. They are not walls: they are internal surfaces that build the `cellZone` and `faceZone` used by `fvOptions`. Their pressure drop still reaches the total drag, through the pressure on the surrounding walls, but it cannot be attributed to a patch.

Seven assemblies cover the 16 wall patches exactly once each, so they add up to the total:

| Function object | Patches |
|---|---|
| `DP_forceCoeffs` | `DP_.*`, the whole car |
| `DP_FW_forceCoeffs` | `DP_FW` |
| `DP_RW_forceCoeffs` | `DP_RW`, `DP_RW_Upper` |
| `DP_Monocoque_forceCoeffs` | `DP_Monocoque`, `DP_Monocoque_Kick`, `DP_Monocoque_Duct`, `DP_Winglets`, `DP_Diffuser`, `DP_T_Tray`, `DP_Radiator_Ducts` |
| `DP_Susp_forceCoeffs` | `DP_Susps`, `DP_Uprights` |
| `DP_UT_forceCoeffs` | `DP_UT` |
| `DP_Wheels_forceCoeffs` | `DP_Wheels_Front`, `DP_Wheels_Rear` |
| `DP_Tyre_Plinth_forceCoeffs` | `DP_Tyre_Plinths` |

Six more function objects isolate single patches for the second tier of the report table: `DP_Winglets`, `DP_RW_Upper`, `DP_T_Tray`, `DP_Diffuser`, `DP_Wheels_Front`, `DP_Wheels_Rear`.

`system/functions/forces` mirrors the same fourteen entries, so every assembly has both its coefficients and its raw forces.

On a converged run the seven assemblies add up to `DP_forceCoeffs` on both `Cd` and `Cl`. That sum is the cheapest check that the patch-to-assembly mapping is still intact after a rename.

---

## Setting up the car

Everything is in `preProcessor/setup.txt`:

```
FRH = 25        front ride height [mm]
RRH = 28        rear ride height [mm]
roll = 0        not used
yaw = 0         not used
steer = 0       front-wheel steer angle [deg]
dZ = 0          front wing height change [mm]
AoA = 0         front wing angle of attack [deg], positive about +y
AttFront, AttRear   front wing mounting brackets, in the OpenFOAM frame
```

`25 / 28` is the DP18 baseline setting. The geometry comes out of CAD at `35 / 35`, which is the reference of the pre-processing: at that setting the car is not pitched at all.

Braking means the nose goes down: lower `FRH`, raise `RRH`. Squat under acceleration is the opposite.

> **Do not add or remove lines in `setup.txt`.** The script reads it **by line number**: `FRH` and `RRH` on lines 3 and 4, `steer` on line 7, `dZ`, `AoA`, `AttFront` and `AttRear` on lines 8 to 11. `roll` and `yaw` on lines 5 and 6 are ignored and stay there so that the line numbers do not shift. Change the values, never the layout.
>
> The same holds for `setup_UBJ_LBJ.txt`, the front suspension hardpoints: the four ball joints are read from lines 8 to 11. Its first lines are placeholders and are ignored.

### How `preProcessor_pietro.py` works

It finds the case root from its own location and works only inside `constant/triSurface/`, which it empties and refills from `constant/triSurface_0deg/`. The CAD files are never modified.

Every transform is rigid, so the whole chain collapses into one 4×4 matrix per body. The script composes it before opening a file, then reads each `.obj` once, applies one matrix and writes once:

```
M(body) = M_pitch · M_FW · M_steer · M_CAD_TO_AERO
```

- `M_CAD_TO_AERO`: the frame change above, applied to everything.
- `M_FW`, only on `FW`: rotation about the rear mounting bracket by the true adjustment angle, plus the `dZ` height change.
- `M_steer`, only on `Wheels_Front`: rotation by `steer` degrees about the right kingpin axis, from `setup_UBJ_LBJ.txt`.
- `M_pitch`, on the sprung mass: rotation about the rear axle by

  ```
  pitch  = -atan((RRH - FRH) / wheelbase)
  zpitch =  RRH/1000 - 0.035
  ```

  Wheels, tyre plinths and uprights are left out: they stay on the ground.

A transform that is the identity at the current setting is dropped from the product, and the script says so on screen.

At the end the script checks it has written **21** files and stops with an error if not.

The knobs are at the top of the file: `BODY` and `BODY_PREFIX` (what follows the sprung mass), `SPLIT_BY_PLANE` and `SPLIT_BY_COMPONENTS`, `SYMMETRY_Y`, `N_SURFACES_EXPECTED`.

### Front-wheel steer

`steer` is `0` in the reference case. Before using it, know what it does and does not do:

- it rotates the **geometry** of `Wheels_Front` only. `Tyre_Plinths` holds all four plinths in one file and is not rotated;
- it does **not** update the wheel rotation centre and axis in `initialConditions`. At a non-zero `steer` or `yaw` the script prints the new `FRCenter`, `FRaxis` and the others on screen, to be pasted into `initialConditions` by hand;
- only the right-hand hardpoints matter, because only `y > 0` is meshed.

---

## Running the case

Every script sources OpenFOAM on its own and reads the processor count from `np` in `initialConditions`. Run them from the case root.

| Script | What it does |
|---|---|
| `./runPreProcess` | frame change, split of the multi-body files, ride height. Writes `constant/triSurface/`. |
| `./runMesh` | pre-processing, then `surfaceFeatureExtract`, `blockMesh`, `decomposePar`, `checkMesh`, `snappyHexMesh`, `checkMesh`, `reconstructParMesh`, `patchSummary`. Logs in `log_mesh/`. |
| `./runSolve` | `decomposePar`, `renumberMesh`, `potentialFoam`, `simpleFoam`, then the post-processing and `reconstructPar`. Logs in `log_solve/`. |
| `./runAll` | mesh and solve in one go. Asks for confirmation twice and shows the free disk space. |
| `./runQueue` | waits for the run in progress on the workstation to end, then meshes and solves. See below. |
| `./runResume` | runs `simpleFoam` again from the latest time, then the post-processing. |
| `./runPostProcess` | post-processing only. |
| `./runClean` | removes time directories, processor folders, logs and `postProcessing/`. The mesh stays. |

`runSolve`, `runAll` and `runQueue` start by deleting the previous solution: time directories, `log_solve/` and `postProcessing/`.

At the end of a run the `processor*` folders are moved into `allProcessors/`. `runResume` and `runPostProcess` need them back in the case root:

```bash
mv allProcessors/processor* .
```

> **`np` must appear only once in `initialConditions`.** The scripts find the processor count with `grep 'np'`, which matches any line containing those two letters, comments included. A second match breaks every `mpirun` call.

### Queueing runs on the workstation

`runSolve`, `runAll`, `runResume` and `runQueue` end by creating the file `/home/$USER/OpenFOAM/pietro-v2512/run/ended.txt`. `runQueue` checks for that file every five minutes; when it appears it deletes it and starts its own mesh and solve.

To queue a run, prepare its case while the previous one is running and launch `./runQueue` in it. Three things to know:

- only **one** case can wait at a time. Two waiting cases would both see the file and start together;
- the file is created even when the previous run failed, so the queued run starts anyway;
- the path is tied to the run folder of the workstation user. On another machine, change it in the five scripts.

---

## Post-processing

During the run, `system/controlDict` executes the function objects in `system/functions/`: force coefficients at every iteration; forces, residuals, mean fields, near-wall velocity, wall shear stress, vorticity, `Q` and `yPlus` at write time. The mean fields `UMean` and `pMean` restart every 100 iterations.

After the solver, the scripts run the same block:

1. `vorticity`, `wallShearStress` and `yPlus` on the latest time;
2. `totalPressureIncompressible` and `mag(vorticity)`;
3. `postProcessor/generate_surfaces.py`, which writes `system/controlDict_surfaces`;
4. `simpleFoam -postProcess -dict system/controlDict_surfaces`, which samples the planes;
5. `reconstructPar` on the latest time.

### `postProcessor/`

`postProcessor/` holds `generate_surfaces.py`. It is not `postProcessing/`: that one is written by OpenFOAM and deleted by `runClean`.

The script writes the dictionary of the cutting planes sampled at the end of the run: 153 planes in `x`, 31 in `y` and 63 in `z`, each with `pMean`, `UMean` and `vorticity`. OpenFOAM writes one VTK file per plane in `postProcessing/surfaces/<time>/`, and aeroTools renders the slices from those files.

```bash
python3 postProcessor/generate_surfaces.py            # all three axes
python3 postProcessor/generate_surfaces.py x z        # only some axes
```

---

## Reading the coefficients

### The library comes first

All fourteen coefficient function objects are `type forceCoeffsModified`, a team-built variant. It is **not** part of OpenFOAM and its sources are not part of this repository: each user compiles it on their own machine, with the same OpenFOAM version the case runs on. It lands in `$FOAM_USER_LIBBIN`, which contains the version in its path, so a library built with a different version is invisible to the solver.

Without it **no coefficient is written at all**.

The warning it produces when missing is misleading and worth recognising:

```
Could not load "forceCoeffsModified.so"
libforceCoeffsModified.so: cannot open shared object file
```

The second line names a file that the build never produces: OpenFOAM retries with a `lib` prefix and reports the error of that second attempt. The library really is called `forceCoeffsModified.so`, without the prefix. The warning also appears from `blockMesh`, `snappyHexMesh` and `patchSummary`, which never build a function object: seeing it there means nothing. The real check is whether `postProcessing/*_forceCoeffs/` fills up.

### Half car, and the moment pole

Forces are those of half a car, with `Aref 1`. **aeroTools scripts double them** before writing `averages.txt`, so the coefficients in that file are full-car values; the ones in `postProcessing/` are not.

The moment pole is `CR (0 0.765 0)` in `initialConditions`. Its `y` does not affect the pitching moment, so the pole is in effect on the **front axle**, at `x = 0`, not at mid-wheelbase. "aeroTools" computes the aerodynamic balance as

```
FB = (1 + CmPitch / Cl) * 100
```

which is only valid with the pole there. Move `CR` to `(0.765 0 0)` and the balance jumps from 48% to 98%. Two consequences:

- `CmPitch`, in `averages.txt` and in `postProcessing/`, is taken about the front axle. It is not the textbook pitching moment coefficient;
- the `Cl(f)` and `Cl(r)` columns written by OpenFOAM in `coefficient.dat` are **meaningless**, because OpenFOAM builds them assuming the pole is at mid-wheelbase. Ignore them and read `FB`.

---

## The mesh

The background mesh has 1 m cells. Each refinement level halves the cell size, so level 9 is about 2 mm and level 11 about 0.5 mm.

All the parameters are in `initialConditions`. Every patch has its own set, named after the patch without the `DP_` prefix:

```
<Patch>_RefLvl              surface refinement level, both entries equal
<Patch>_feature             feature edge level, same as RefLvl
<Patch>_nLayers             number of layers
firstLayerThickness_<Patch> first layer thickness [m]
expansionRatio_<Patch>      expansion ratio
```

You tune one patch at a time without touching the others. The first layer thicknesses are sized for wall functions, with a target `y+` of about 60.

### Refinement around the car

Each wall patch is surrounded by a cascade of refinement bands, each one a `(distance level)` pair. There are three cascades:

| Tier | Variables | Patches | From → to |
|---|---|---|---|
| General | `dist1`-`dist6`, `ref1`-`ref6` | every wall patch not listed below | level 9 within 15 mm → level 4 within 1 m |
| Monocoque | `dist1Mono`-`dist3Mono`, `ref1Mono`-`ref3Mono` | `Monocoque`, `Monocoque_Kick`, `Susps`, `Uprights` | level 8 within 150 mm → level 6 within 300 mm |
| Duct | `dist1Mono_duct`-`dist3Mono_duct`, `ref1Mono_duct`-`ref3Mono_duct` | `Monocoque_Duct` | level 9 within 50 mm → level 7 within 200 mm |

Fourteen boxes, `min1`/`max1`/`lvl1` to `min14`/`max14`/`lvl14`, refine the wheel wakes, the volume around the car, the wake behind it, the ground and the fans. Box 9 is a cylinder: `min9` and `max9` are the two points of its axis and `radius9` its radius.

### Layers

Layers are added with `meshShrinker displacementMotionSolver` and the `displacementLaplacian` solver, set in `addLayersControls` of `snappyHexMeshDict`. That solver needs the `cellDisplacement` entries in `system/fvSchemes` and `system/fvSolution`: they are not leftovers, and `snappyHexMesh` stops without them.

Thicknesses are absolute. The only exception is `ground`, which uses a relative thickness.

### Arity matters

Two families of variables look interchangeable and are not:

| Variable | Shape | Consumed by | Wants |
|---|---|---|---|
| `<Patch>_RefLvl` | pair `(min max)` | `refinementSurfaces/<s>/level` | `labelPair` |
| `<Patch>_feature` | scalar | `features/{ level ... }` | `label` |
| `Porous_regionLvl`, `Fan_regionLvl` | scalar | `refinementRegions/<s>/levels` | see below |

`levels` in `refinementRegions` is a list of `(distance level)` **pairs**, in every mode. Putting a `_RefLvl` pair there expands into a pair of pairs and `snappyHexMesh` dies reading the dictionary, before it meshes anything, with *wrong token type - expected Scalar, found the punctuation token `(`*. Hence the two dedicated scalars.

In `mode inside` the distance is read and then discarded, so every such entry is written the same way, `levels ((1e15 $X));`, with `1e15` as a placeholder. The five zones (`Porous_Battery`, `Porous_Radiators`, `Fan_Battery_1`, `Fan_Battery_2`, `Fan_Radiators`) and the fourteen boxes use `mode inside`; the `DP_*` wall patches use `mode distance`. No surface carries two entries: OpenFOAM merges sub-dictionaries with the same name and the second one silently wins.

On the first mesh of a new geometry, read the layer coverage table at the end of `log_mesh/05_snappyHexMesh` and the quality metrics in `log_mesh/06_checkMesh`, and tune from there.

---

## Adding or removing a patch

A patch lives in **eight places**, and all eight have to move together:

| File | Places per patch |
|---|---|
| `system/snappyHexMeshDict` | 5: `geometry`, `features`, `refinementSurfaces`, `refinementRegions`, `layers` |
| `system/surfaceFeatureExtractDict` | 1 |
| `system/functions/forceCoeffs` | 1 |
| `system/functions/forces` | 1 |

Missing one raises no error on screen: the patch is simply not refined, gets no layers, or does not show up in `averages.txt`. To add one:

1. put the `.obj` in `constant/triSurface_0deg/`;
2. add the five blocks in `snappyHexMeshDict` and the block in `surfaceFeatureExtractDict`;
3. add its five parameters in `initialConditions`;
4. add it to an assembly in `forceCoeffs` and `forces`, and give it a function object of its own if you want it in the report;
5. if it belongs to the sprung mass, add it to the `BODY` list in `preProcessor_pietro.py`, otherwise it will not follow the car pitch;
6. update `N_SURFACES_EXPECTED` in the same file.

Check with:

```bash
grep -n "<PatchName>" system/snappyHexMeshDict system/surfaceFeatureExtractDict system/functions/force* initialConditions
```

After meshing, `log_mesh/09_patchSummary` must list the patch with a non-zero face count.

---

## Porous media and fans

The pressure drops and the fans are configured in `system/fvOptions`:

| Entry | Type | Zone |
|---|---|---|
| `Radiator` | `explicitPorositySource`, Darcy-Forchheimer | `Porous_Radiators` |
| `Battery` | `explicitPorositySource`, Darcy-Forchheimer | `Porous_Battery` |
| `Radiator_fan` | `fanMomentumSource` | `Fan_Radiators` |
| `Battery_fan1` | `fanMomentumSource` | `Fan_Battery_1` |
| `Battery_fan2` | `fanMomentumSource` | `Fan_Battery_2` |

The zones come from `snappyHexMeshDict`, where those surfaces are declared with `faceZone`, `cellZone` and `cellZoneInside inside`. Keep the names consistent between the two dictionaries.

The fan curves are `constant/fanCurve_Battery.txt` (San Ace 60 9GA0624P1J03) and `constant/fanCurve_Radiator.txt` (ebm-papst 4118 N/2H8P). Each is the curve of a **single** fan, which is why the battery disks are never merged into one zone: a zone holding two disks would read the curve at twice the flow rate. Update the files when the hardware changes.

If a source does not react, check that its zone is not empty after `snappyHexMesh`: `constant/polyMesh/cellZones` must list it with a cell count greater than zero.

---

## Repository layout

- `constant/triSurface_0deg/`: **the geometries**, from CAD, in the CAD frame. This is the only folder you touch when new geometry arrives.
- `constant/`: `transportProperties`, `turbulenceProperties` and the two fan curves.
- `system/`: mesh and solver dictionaries, plus `functions/` for the function objects.
- `orig0/`: boundary and initial conditions (`U`, `p`, `k`, `omega`, `nut`). The scripts copy it into `0/`.
- `initialConditions`: the parameters shared by almost every dictionary: velocity, processor count, iterations, refinement levels, layers, wheel centres.
- `preProcessor/`: `preProcessor_pietro.py`, `setup.txt` and `setup_UBJ_LBJ.txt`.
- `postProcessor/`: `generate_surfaces.py`.
- `caseInfo.json`: run metadata, to fill in for each run.
- `DP.foam`: empty file to open the case in ParaView.
- `runPreProcess`, `runMesh`, `runSolve`, `runAll`, `runQueue`, `runResume`, `runPostProcess`, `runClean`.

Everything a run generates is listed in `.gitignore` and stays out of the repository: `0/` and the time directories, `constant/triSurface/`, `constant/polyMesh/`, `Mesh/`, the processor folders, the logs, `postProcessing/` and `system/controlDict_surfaces`.

`.gitattributes` forces Unix line endings. A script saved with Windows line endings stops on its first line when run on Linux.

---

## Troubleshooting

- **`snappyHexMesh` dies before meshing, on a dictionary read.** Almost always an arity mistake in `initialConditions`, see [Arity matters](#arity-matters). The message names the entry and the line.
- **`mpirun` fails in every step with an invalid process count.** `np` appears more than once in `initialConditions`, or a script has Windows line endings.
- **`cannot find file ".../system/controlDict_surfaces"`.** `postProcessor/generate_surfaces.py` failed just before: its error is a few lines above on screen.
- **`runResume` or `runPostProcess` finds no processor folders.** They are in `allProcessors/`: move them back to the case root.
- **A patch is missing from `averages.txt`.** Check it is active in all eight places, that the mesh was rebuilt, and that `postProcessing/<patch>_forceCoeffs/` exists. aeroTools skips missing folders without an error, so a forgotten patch disappears silently.
- **The car is in the wrong place.** Open one `.obj` from `constant/triSurface/` in ParaView. Front axle at `x = 0`, rear axle at `x = 1.53`. If the car sits around `x = 3`, the geometry was transformed before being put in `triSurface_0deg/`. The geometry is the **whole** car and extends to `y < 0`: that is correct, the domain starts at `y = 0` and `snappyHexMesh` ignores the rest.
- **A wheel spins far too fast.** The split did not work. `Wheels_Front.obj` and `Wheels_Rear.obj` must both exist, each holding the wheels of one axle.
- **No coefficient comes out.** See [The library comes first](#the-library-comes-first): nine times out of ten it is `forceCoeffsModified.so`.

To look at bad cells in ParaView:

```bash
checkMesh -allGeometry -allTopology -writeAllFields -writeSets vtk
foamToVTK -faceSet nonOrthoFaces -time 0
```
