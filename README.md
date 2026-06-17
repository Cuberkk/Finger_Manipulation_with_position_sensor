# Finger Manipulation Experiment Protocol
Force Sensor 1(NIDAQ): FT44298 </br>
Force Sensor 2(NIDAQ): FT45281 </br>
Force Sensor 3(Labjack): FT44297 </br>

Finger positioning for each finger
<ul>
    <li> Thumb for Sensor 1</li>
    <li> Index for Sensor 3</li>
    <li> Middle for Sensor 2</li>
</ul>

For data access, please refer to:
`data/{rotation axis}/test_{number}`

The data structure of `raw_data.csv` is:</br>
$\textbf{[}S_1-F_x, S_1-F_y, S_1-F_z, S_1-\tau_x, S_1-\tau_y, S_1-\tau_z,$</br>
$S_2-F_x, S_2-F_y, S_2-F_z, S_2-\tau_x, S_2-\tau_y, S_2-\tau_z,$</br>
$S_3-F_x, S_3-F_y, S_3-F_z, S_3-\tau_x, S_3-\tau_y, S_3-\tau_z \textbf{]}$

The data structure of `transformed_data.csv` is:</br>
$\textbf{[}S_1-F_x, S_1-F_y, S_1-F_z, S_1-\tau_x, S_1-\tau_y, S_1-\tau_z,$</br>
$S_2-F_x, S_2-F_y, S_2-F_z, S_2-\tau_x, S_2-\tau_y, S_2-\tau_z,$</br>
$S_3-F_x, S_3-F_y, S_3-F_z, S_3-\tau_x, S_3-\tau_y, S_3-\tau_z \textbf{]}$

While these components are <strong>zeros</strong>:</br>
 $\textbf{[}S_1-\tau_x, S_1-\tau_y, S_1-\tau_z,$</br>
$S_2-\tau_x, S_2-\tau_y, S_2-\tau_z,$</br>
$S_3-\tau_x, S_3-\tau_y, S_3-\tau_z \textbf{]}$

A PCA script can be found in `data_analysis.ipynb`.</br>
When performing PCA on combined force and torque data, `StandardScaler` is applied prior to PCA to normalize the feature scales.</br>
For force-only analysis, scaling is disabled to preserve the physical magnitude relationships.

## Finger Force to Finger Position Frame Calculations
Fingers 1 & 2 refer to NIDAQ_ATI_publisher.py</br>
Finger 3 refer to Labjack_ATI_publisher.py
<i><p style= "font-size: 17px;" > Finger Force 1 to Finger  Postion 1 Rotations: </br>
</i> <p style= "font-size: 13px;" > 
<b>F<sub>finger1</sub></b>
=(
R<sup>T</sup><sub>base←finger1</sub>
R<sub>base←obj</sub>
R<sub>z</sub>(120°)
R<sub>base</sub>
R<sub>z</sub>(48°)
)
<b>F<sub>force1</sub></b>
<i><p style= "font-size: 17px;" > Finger Force 2 to Finger Position 2 Rotations: </br>
</i><p style= "font-size: 13px;" > 
<b>F<sub>finger2</sub></b>
=(
R<sup>T</sup><sub>base←finger2</sub>
R<sub>base←obj</sub>
R<sub>z</sub>(0°)
R<sub>base</sub>
R<sub>z</sub>(48°)
)
<b>F<sub>force2</sub></b> </br>
<i> <p style= "font-size: 17px;" > Finger Force 3 to Finger Position 3 Rotations: 
</i> <p style= "font-size: 13px;" > 
<b>F<sub>finger3</sub></b>
=(
R<sub>x</sub>(180°)
R<sup>T</sup><sub>base←finger3</sub>
R<sub>base←obj</sub>
R<sub>z</sub>(-120°)
R<sub>base</sub>
R<sub>z</sub>(48°)
)
<b>F<sub>force3</sub></b>

## Error Calculations
For finger to object refer to relative_rotational_error_node.py</br>
For finger to base refer to rotational_error_node.py

<i><p style= "font-size: 17px;" > Calibrating Finger Postion Sensor to Object Postion Sensor Rotations: </br>
</i> <p style= "font-size: 13px;" > 
<b>R<sub>finger←object</sub></b>
=<b>R</b><sup>T</sup><sub>base←finger</sub>
<b>R</b><sub>base←object</sub>
<p style= "font-size: 13px;" > 
<b>Error</b>
=tr(
<b>R<sub>finger←object</sub></b>
)
− 3
<i><p style= "font-size: 17px;" > Calibrating Finger Position Sensor to Polhemus Base Rotations: </br>
</i> <p style= "font-size: 13px;" > 
<b>R<sub>error</sub></b>
=<b>R</b><sup>T</sup><sub>GT</sub>
<b>R</b><sub>base←sensor</sub>
<p style= "font-size: 13px;" > 
<b>Error</b>
=tr(
<b>R<sub>error</sub></b>
)
− 3


![alt text](data/roll_pitch_yaw.png)