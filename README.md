# Finger Manipulation Experiment Protocol
Force Sensor 1(NIDAQ): FT44298 </br>
Force Sensor 2(NIDAQ): FT45281 </br>
Force Sensor 3(Labjack): FT44297 </br>

Finger positioning for each finger
<ul>
    <li> Thumb for Sensor 1</li>
    <li> Index for Sensor 3</li>
    <li> Middle for Senosr 2</li>
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



![alt text](data/roll_pitch_yaw.png)