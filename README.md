# Finger Manipulation Experiment Protocol

## Repository Layout

All acquisition and experiment functionality lives in `utils/`, grouped by purpose. Recorded and generated data lives in `data/`; analysis notebooks live in `data_process/`.

```text
utils/
├── acquisition/              # NI-DAQ and LabJack device readers
├── publishers/               # ROS 2 force publishing and frame transformations
├── recording/                # CSV recorders
├── calibration/              # Position sensor rotational error nodes
├── visualization/            # RViz object-axis visualization
├── gui/                      # Researcher and participant interface
├── object_meshes/            # Final 50mm.stl and 80mm.stl used by the GUI
├── launch/                   # Multi-recorder launcher
├── config/calibration_files/ # ATI .cal files
└── paths.py                  # Shared repository, data, and calibration paths
data/
├── user_*/                   # Participant recordings
├── sensor_bias/              # Sensor bias CSV and JSON files
├── synced_data/              # Synchronized analysis data
├── segmented_data/           # Segmented analysis data
└── analysis_results/         # PCA figures and other analysis outputs
data_process/                # Single-trial and multi-trial analysis notebooks
docs/images/                 # Protocol reference images
docs/mesh_sources/           # Archived original high-resolution STL models
```

Existing experiment folders under `data/` retain their layout.

## Running the Experiment

Run the following commands from the repository root using the existing Python environment with ROS 2, the Polhemus workspace, and the NI-DAQ/LabJack drivers configured. The publishers require the corresponding hardware; finger-frame transformations also require the Polhemus TF stream. Source the ROS/workspace setup scripts as usual before starting nodes.

Start each publisher in its own terminal:

```bash
python -m utils.publishers.NIDAQ_ATI_publisher
python -m utils.publishers.Labjack_ATI_publisher
python -m utils.publishers.Object_Origin_ATI_publisher --ros-args -p diameter_mm:=50
```

Set `diameter_mm` to the object being used (`50` or `80`). The object-origin publisher subscribes to the raw force topics from the two hardware publishers.

Record a trial with one of the following commands, or use the launcher below to start all three recorders:

```bash
python -m utils.recording.force_trial_csv_recorder -u 1 -s 50 -t roll -tn 1
python -m utils.recording.force_raw_csv_recorder -u 1 -s 50 -t roll -tn 1
python -m utils.recording.object_origin_csv_recorder -u 1 -s 50 -t roll -tn 1

python -m utils.launch.recoder_launch -u 1 -s 50 -t roll -tn 1
```

The launcher opens three GNOME Terminal windows and requires `gnome-terminal`. The existing `recoder_launch` spelling is retained.

| Argument | Meaning |
| --- | --- |
| `-u`, `--user-number` | Participant number |
| `-s`, `--diameter-mm` | Object diameter: `50` or `80` |
| `-t`, `--task-performed` | `roll`, `pitch`, `yaw`, `finger_gating`, or `single_fg` |
| `-tn`, `--trial-number` | Trial number |
| `--duration-sec` | Recording duration; default `30`, or `0` until Ctrl+C |
| `--data-root` | Override the default repository `data/` directory |
| `--flush-every` | Flush CSV files after this many samples; default `25` |
| `--sensor-subfolders` | Put each finger's CSV in a separate sensor folder |
| `--overwrite` | Delete the existing trial folder before recording; otherwise append |

Default data and calibration paths are resolved from the repository location, independent of the working directory. The module commands above still require the repository root on Python's module search path. An explicitly supplied relative `--data-root` is relative to the working directory.

To use the researcher/participant GUI:

```bash
python -m utils.gui.study_gui --demo
python -m utils.gui.study_gui
```

The GUI opens separate researcher and participant windows. Demo mode runs with Tkinter and NumPy on a graphical desktop, generates simulated force/torque and tracking data, and saves it under `data/demo/` by default. Live mode uses the running hardware publishers and Polhemus TF stream. The GUI records its own data; the standalone recorders are not needed for this workflow.

In **demo mode**, the advisor has a **DEMO · DATA SAVING** row directly below the User Number / Trial controls at the top. Its **Save data (CSV + metadata)** checkbox is checked by default. Uncheck it to test the complete workflow without creating session folders, CSV files, or metadata files. The choice applies to the next trial and is locked during preparation, recording, and saving; it can be changed during rest. The interface explicitly labels trials with recording disabled as **DEMO PREVIEW**. Live mode always records data. Restart the GUI after updating the code, using `python -m utils.gui.study_gui --demo`, to load the current interface.

Demo participants can use **Space** to move through stages quickly: during the five-second preparation countdown it starts the trial immediately; during cycle counting it finishes the trial as `demo_skipped`; during rest it skips the remaining rest and opens the next trial's preparation countdown. Each press advances one stage, so holding Space does not skip multiple stages. Saving still finishes before progression. Recorded demo trials retain the actual completed cycle count and store `demo_cycle_count_skipped: true` for skipped counting; skips never fabricate 20 completed rotations. In live/formal mode, Space can skip rest only; preparation still lasts five seconds and trials still require all 20 cycles.

Press **Ctrl+C in the GUI's terminal** or close the advisor window to end the session. The GUI stops recording, processes the remaining recorded orientations, writes all queued CSV rows and final trial metadata, and waits for its recording/acquisition workers and ROS resources to close before destroying the windows. Repeated Ctrl+C presses during saving do not interrupt this cleanup. SIGTERM follows the same save path. An unfinished trial receives `result: "interrupted"` for Ctrl+C, `"terminated"` for SIGTERM, or `"closed"` for window closure; already completed trials keep `"completed"`. Interrupting the countdown creates no trial recording. The terminal reports the saved directory or any save error.

Publishers and recorders started in separate terminals remain independent processes; stop them in their own terminals when finished. The GUI does not own or stop them.

The trial selector follows [Trials_lookup.csv](docs/user_study_docs/Trials_lookup.csv): 40 measured trials and 8 practice trials, covering 50 mm and 80 mm objects. Measured references retain the lookup's `00`–`39` numbering; practice references are `P00`–`P07`. Each task/diameter group has five measured repetitions, displayed as `Task 1/5` through `Task 5/5`.

The participant window has a white background and displays trial/task information, overall progress, the object mesh, and the target axis. It omits the STL caption, individual rotation-phase instructions, and the five-step sequence strip. The selected 50 mm or 80 mm mesh follows sensor 4 orientation relative to a captured baseline. Roll, pitch, and yaw use the fixed mesh X, Y, and Z axes respectively.

With the participant window focused, press **Space** to start the selected trial. After completing 20 cycles and saving the data, the participant automatically enters a rest period. In either mode, press **Space** during rest to skip its remainder and enter the next trial's preparation countdown. When rest finishes naturally, press **Space** when ready to proceed. The participant display shows a **five-second circular countdown** before each start; it captures a fresh orientation baseline when the countdown ends. The advisor's Start button uses the same countdown and can cancel it. In live mode, Space is ignored during preparation, recording, and saving; holding Space does not submit repeated requests. Early stops and save errors require the researcher to select or restart a trial. After the last trial, the participant sees a completion message.

Rest durations come from [Rest_Time.py](docs/user_study_docs/Rest_Time.py), whose values are in **minutes**. The current settings are 60 seconds between trials, 120 seconds between tasks, and 300 seconds between objects. Only one duration applies at a boundary: changing objects takes priority over changing tasks, then ordinary trial rest. Practice-to-measured transitions within the same task/object use the ordinary trial rest. Both preparation and rest use the same circular display, with only the integer remaining seconds in its center. No elapsed / total time appears below the ring. Rest finishes on `0` with a Space prompt. The first trial and the end of the study have no rest period. Advisor trial-start/navigation controls wait until rest finishes or the participant skips it with Space; Ctrl+C still saves and closes normally.

Edit the minute values and restart the GUI to change rest durations, or use `--rest-config PATH` with an alternative configuration containing `trials`, `tasks`, and `objects`. A zero value omits that rest; fractional seconds round up to whole seconds. Each session's metadata records the loaded rest configuration and durations.

Counting requires a complete rotation cycle: **neutral → negative target → neutral → positive target → neutral**. The researcher can adjust the default targets of ±30°, neutral band of ±5°, and 0.12-second hold requirement. The trial stops automatically at 20 cycles; stopping early also saves the recorded data. Tracking interruptions discard an unfinished cycle while retaining completed cycles. The `G` / finger-gating task currently uses the researcher's manual `+1` count pending a defined automatic counting rule.

For example, `80R0` means the 80 mm object, Roll, repetition 0; it is displayed as `Task 1/5`. Move the participant window to its own monitor if needed. Both windows scale their fonts independently when resized or maximized using the window controls. F11 toggles fullscreen in either window; Escape leaves fullscreen. Restoring the window size restores its font sizes.

The advisor's three charts show Thumb, Index, and Middle, each with six signed bars and separate force/torque scales. By default, starting a trial requires fresh sensor-4 tracking and all three six-component F/T streams. Interrupted or missing F/T data and recorder queue overflows are flagged for review; stale bars are not presented as current measurements.

`utils/object_meshes/` contains only the two final meshes used by the GUI: `50mm.stl` and `80mm.stl`, with corrected winding/normals and prepared display geometry. The original high-resolution models are archived in `docs/mesh_sources/`. Runtime loading uses the final files directly. To regenerate either final mesh from its archived source:

```bash
python -m utils.gui.mesh_repair docs/mesh_sources/50mm_original.stl utils/object_meshes/50mm.stl --overwrite
python -m utils.gui.mesh_repair docs/mesh_sources/80mm_original.stl utils/object_meshes/80mm.stl --overwrite
```

The preparation command saves one final STL and preserves its source. Preparing dense meshes requires VTK, available in the system Python environment on this workstation. VTK is only needed for preparation; the GUI continues to use Tkinter and NumPy at runtime.

GUI configuration options:

| Argument | Meaning |
| --- | --- |
| `--demo` | Generate simulated samples; default save root is `data/demo/` |
| `--data-root` | Override the save root; live mode defaults to repository `data/` |
| `--protocol` | Override the trial lookup CSV |
| `--rest-config` | Rest_Time.py-style configuration; values are minutes |
| `--mesh-dir` | Directory containing the 50 mm and 80 mm STL models |
| `--timezone` | Time zone used for session and trial folder timestamps |
| `--mesh-alignment-rpy RX RY RZ` | Mesh-to-sensor-4 alignment in degrees, with `A = Rz @ Ry @ Rx`; default is identity |

The displayed and measured rotation is `A.T @ R0.T @ Rt @ A`, where `R0` is the baseline sensor-4 rotation and `Rt` is its current rotation. Alignment is explicit; the GUI does not apply an implicit axis permutation.

GUI recordings use a session/task/trial structure:

```text
data/
└── User001_MDY_HM/
    ├── session_metadata.json
    └── Roll/
        └── User001_Trial20_MDY_HM/
            ├── thumb.csv
            ├── index.csv
            ├── middle.csv
            ├── thumb_raw_force.csv
            ├── index_raw_force.csv
            ├── middle_raw_force.csv
            ├── object_orientation.csv
            ├── rotation_progress.csv
            └── trial_metadata.json
```

`MDY_HM` means `%m%d%Y_%H%M`, for example `10232026_1430`. Session and trial timestamps describe their respective start times. Task folders are `Roll`, `Pitch`, `Yaw`, and `FingerGaiting`. Repeated session/trial names within the same minute receive a suffix such as `_02` to preserve existing recordings. Practice trials retain their `Pxx` reference in the trial folder name.

Each finger CSV contains `source_timestamp_sec, Fx, Fy, Fz, Tx, Ty, Tz`, with force in N and torque in N·mm. The matching raw files receive calibrated, bias-corrected sensor-frame samples when those topics are available. `object_orientation.csv` stores sensor-4 timestamps and `qx, qy, qz, qw`; `rotation_progress.csv` stores timestamps, elapsed time, signed angle, completed cycles, and counter phase. Session/trial JSON files store the experiment settings and recording details.

For six-component finger-frame data, the hardware publishers provide `/ERIE_Manipulation/force/wrench_s1_finger1` (thumb), `wrench_s3_finger3` (index), and `wrench_s2_finger2` (middle), all under the same `/ERIE_Manipulation/force/` prefix. The existing `force_sN_fingerN` force-only topics remain available. Both force and torque are rotated into the finger-frame axes, but torque retains the ATI measurement origin: no `r × F` moment-transfer term is applied. These torque values therefore do not represent moments transferred to the finger sensor or object origin.

Calibration and visualization nodes are available as modules:

```bash
python -m utils.calibration.rotational_error_node
python -m utils.calibration.relative_rotational_error_node
python -m utils.visualization.object_axis_rviz_visualizer
```

## Sensors and Data

| Sensor | Interface | ATI calibration | Finger |
| --- | --- | --- | --- |
| 1 | NI-DAQ | FT44298 | Thumb |
| 2 | NI-DAQ | FT45281 | Middle |
| 3 | LabJack | FT44297 | Index |

Use the same finger-to-sensor mapping for position sensor calibration.

Standalone recorder trial recordings are stored in:

```text
data/user_{number}/{diameter}mm/{task}/trial_{number}/
```

| Recorder | CSV filenames | CSV columns |
| --- | --- | --- |
| [Finger-frame force](utils/recording/force_trial_csv_recorder.py) | `thumb.csv`, `middle.csv`, `index.csv` | `source_timestamp_sec, Fx, Fy, Fz` |
| [Raw sensor force/torque](utils/recording/force_raw_csv_recorder.py) | `thumb_raw_force.csv`, `middle_raw_force.csv`, `index_raw_force.csv` | `source_timestamp_sec, Fx, Fy, Fz, Tx, Ty, Tz` |
| [Object-origin pipeline](utils/recording/object_origin_csv_recorder.py) | `object_origin_thumb_force.csv`, `object_origin_middle_force.csv`, `object_origin_index_force.csv` | `source_timestamp_sec, Fx, Fy, Fz` |

Here, raw sensor values have already been calibrated and bias-corrected. The object-origin pipeline currently publishes contact-frame force after the object-origin rotation and contact-position calculation; its existing topic and file names are preserved. The GUI's separate session layout and seven-column force/torque format are described above.

Legacy datasets may use `data/{rotation axis}/test_{number}/` with `raw_data.csv` and `transformed_data.csv`. Those older combined files contain three consecutive sensor blocks `[Fx, Fy, Fz, Tx, Ty, Tz]`; the transformed torque columns are zero placeholders. The current recorders use the per-finger formats shown above.

## Data Analysis

Start Jupyter from the repository root or `data_process/`, and open the appropriate notebook:

- [Single-trial Polhemus force analysis](data_process/single_case_polhemus_force_data_analysis.ipynb)
- [Multi-trial Polhemus force analysis](data_process/serial_polhemus_force_data_analysis.ipynb)
- [Single-trial object-origin pipeline analysis](data_process/single_case_object_origin_force_data_analysis.ipynb)
- [Multi-trial object-origin pipeline analysis](data_process/serial_object_origin_force_data_analysis.ipynb)
- [Exploratory notebook](data_process/test.ipynb)

Run the path setup cell first, then set the participant, diameter, task, and trial variables in the notebook. Paths resolve to the repository's `data/` directory. Synchronized and segmented files remain under `data/`, and exported PCA figures are saved to `data/analysis_results/`.

The notebooks currently build paths for the standalone recorders' `data/user_{number}/{diameter}mm/{task}/trial_{number}/` layout. To analyze a new GUI recording, update the notebook's input paths to the selected session/task/trial folder.

In the single-trial PCA routines, combined force and torque data is normalized with `StandardScaler` before PCA. Force-only analysis leaves scaling disabled to preserve the physical magnitude relationships.

## Finger Force to Finger Position Frame Calculations
Fingers 1 & 2 refer to [NIDAQ_ATI_publisher.py](utils/publishers/NIDAQ_ATI_publisher.py). </br>
Finger 3 refers to [Labjack_ATI_publisher.py](utils/publishers/Labjack_ATI_publisher.py).
<b><i><p style= "font-size: 16px;" > Finger Force 1 to Finger  Postion 1 Rotations: </b> </br>
</i> <p style= "font-size: 13px;" > 
<b>F<sub>finger<sub>1</sub></sub></b>
=(
R<sup>T</sup><sub>base←finger<sub>1</sub></sub>
R<sub>base←obj</sub>
R<sub>force<sub>2</sub>'←force<sub>1</sub>'</sub>(120°)
R<sub>object←force<sub>1</sub>'</sub>
R<sub>force<sub>1</sub>'←force<sub>1</sub></sub>(48°)
)
F<sub>force<sub>1</sub></sub>
<b><i><p style= "font-size: 16px;" > Finger Force 2 to Finger Position 2 Rotations:</b> </br>
</i><p style= "font-size: 13px;" > 
<i><b>F<sub>finger<sub>2</sub></sub></b>
=(
R<sup>T</sup><sub>base←finger<sub>2</sub></sub>
R<sub>base←obj</sub>
R<sub>force<sub>2</sub>'←force<sub>2</sub>'</sub>(0°)
R<sub>object←force<sub>2</sub>'</sub>
R<sub>force<sub>2</sub>'←force<sub>2</sub></sub>(48°)
)
F<sub>force<sub>2</sub></sub> </i>
<b> <i> <p style= "font-size: 16px;" > Finger Force 3 to Finger Position 3 Rotations: </b>
</i> <p style= "font-size: 13px;" > 
    <li> Case 1: Includes R<sub>x</sub>(180°) to account for large polhemus sensor </br> </li>

</i> <p style= "font-size: 13px;" > 
<i><b>F<sub>finger<sub>3</sub></sub></b>
=(
R<sub>x</sub>(180°)
R<sup>T</sup><sub>base←finger<sub>3</sub></sub>
R<sub>base←obj</sub>
R<sub>force<sub>2</sub>'←force<sub>3</sub>'</sub>(-120°)
R<sub>object←force<sub>3</sub>'</sub>
R<sub>force<sub>3</sub>'←force<sub>3</sub></sub>(48°)
)
F<sub>force<sub>3</sub></sub> </i>

<li> Case 2:Without R<sub>x</sub>(180°) </br>

</i> <p style= "font-size: 13px;" > 
<i><b>F<sub>finger<sub>3</sub></sub></b>
=(
R<sup>T</sup><sub>base←finger<sub>3</sub></sub>
R<sub>base←obj</sub>
R<sub>force<sub>2</sub>'←force<sub>3</sub>'</sub>(-120°)
R<sub>object←force<sub>3</sub>'</sub>
R<sub>force<sub>3</sub>'←force<sub>3</sub></sub>(48°)
)
F<sub>force<sub>3</sub></sub> </i>

## Finger Force to Object Origin Frame Calculations
Refer to [Object_Origin_ATI_publisher.py](utils/publishers/Object_Origin_ATI_publisher.py).
<b><i><p style="font-size: 16px;">Finger Force 1 & Torque 1 to Object Origin Frame Rotation:</b></i>
<p style="font-size: 13px;">
<b>F<sub>obj_origin₁</sub></b> = R<sub>obj_origin←force</sub>(42°) F<sub>force₁</sub>&nbsp;&nbsp;&nbsp; </br>
<b>τ<sub>obj_origin₁</sub></b> = R<sub>obj_origin←torque</sub>(42°) τ<sub>torque₁</sub>

<b><i><p style="font-size: 16px;">Finger Force 2 & Torque 2 to Object Origin Frame Rotation:</b></i>
<p style="font-size: 13px;">
<b>F<sub>obj_origin₂</sub></b> = R<sub>obj_origin←force</sub>(42°) F<sub>force₂</sub>&nbsp;&nbsp;&nbsp; </br>
<b>τ<sub>obj_origin₂</sub></b> = R<sub>obj_origin←torque</sub>(42°) τ<sub>torque₂</sub>

<b><i><p style="font-size: 16px;">Finger Force 3 & Torque 3 to Object Origin Frame Rotation:</b></i>
<p style="font-size: 13px;">
<b>F<sub>obj_origin₃</sub></b> = R<sub>obj_origin←force</sub>(42°) F<sub>force₃</sub>&nbsp;&nbsp;&nbsp; </br>
<b>τ<sub>obj_origin₃</sub></b> = R<sub>obj_origin←torque</sub>(42°) τ<sub>torque₃</sub>


<b><i><p style="font-size: 16px;">Contact Point Solution (θ, h):</b></i>
<p style="font-size: 13px;">
Given <b>F<sub>object<sub>i</sub></sub></b> = (f<sub>x</sub>′, f<sub>y</sub>′, f<sub>z</sub>′) and <b>τ<sub>object<sub>i</sub></sub></b> = (τ<sub>x</sub>′, τ<sub>y</sub>′, τ<sub>z</sub>′): </br>
φ = atan2(f<sub>x</sub>′, f<sub>z</sub>′) </br>
θ<sub>i</sub> = arcsin( −(l·f<sub>x</sub>′ + τ<sub>y</sub>′) / (R·√(f<sub>x</sub>′²+f<sub>z</sub>′²)) ) + φ </br>
h<sub>i</sub> = H + (τ<sub>z</sub>′ − f<sub>y</sub>′·R·sin θ<sub>i</sub>) / f<sub>x</sub>′
<p style="font-size: 12px;">
where l is the fixed lever offset from sensor origin to object origin, <i>R</i> is the object (cylinder) radius, and <i>H</i> is the sensor origin height offset.


<b><i><p style="font-size: 16px;">Finger Force to Contact Frame Rotation:</b></i>
<p style="font-size: 13px;">
<b>F<sub>contact<sub>i</sub></sub></b> = R<sub>contact←object</sub>(θ<sub>i</sub>) F<sub>object<sub>i</sub></sub>
<p style="font-size: 12px;">
R<sub>contact←object</sub>(θ) =
[ 0&nbsp;&nbsp;&nbsp;1&nbsp;&nbsp;&nbsp;0 ;
−cos θ&nbsp;&nbsp;&nbsp;0&nbsp;&nbsp;&nbsp;sin θ ;
sin θ&nbsp;&nbsp;&nbsp;0&nbsp;&nbsp;&nbsp;cos θ ]

## Error Calculations
For finger to object refer to [relative_rotational_error_node.py](utils/calibration/relative_rotational_error_node.py).</br>
For finger to base refer to [rotational_error_node.py](utils/calibration/rotational_error_node.py).

<b><i> <p style= "font-size: 16px;" > Calibrating Finger Postion Sensor to Object Postion Sensor Rotations: </b></br>
</i> <p style= "font-size: 12px;" > 
<i><b>R<sub>finger←object</sub></b>
=R<sup>T</sup><sub>base←finger</sub>
R<sub>base←object</sub>
<p style= "font-size: 12px;" > 
<b>Error</b>
=tr(
R<sub>finger←object</sub>
)
− 3 </i>
<b><i><p style= "font-size: 16px;" > Calibrating Finger Position Sensor to Polhemus Base Rotations: </b> </br>
</i> <p style= "font-size: 12px;" > 
<i><b>R<sub>error</sub></b>
=R<sup>T</sup><sub>GT</sub>
R<sub>base←sensor</sub> 
<p style= "font-size: 12px;" > 
<b>Error</b>
=tr(
R<sub>error</sub>
)
− 3

## Reference 
![Roll, pitch, and yaw reference](docs/images/roll_pitch_yaw.png)
