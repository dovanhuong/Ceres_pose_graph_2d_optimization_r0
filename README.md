<h1>Ceres_pose_graph_2d_optimization_r0</h1>
<br>
<strong> Steps instructions: </strong>
<br>
<strong> Step 1: </strong>
<br> Compile Ceres optimization library in Visual studio.
<br>http://ceres-solver.org/<br>
<strong> Step 2: </strong>
<br> Add boost C++ libraries from the link as below.<br>
https://www.boost.org/doc/libs/1_61_0/libs/algorithm/doc/html/index.html <br>
<strong> Step 3: </strong>
<br> Compile the code from CMake and build at Visual Studio in Release.<br>
<strong>Step 4: </strong>
<br> Output is the file <i>"optimized_nodes.txt" </i> and <i>"optimized_edges.txt"</i>
Execute python script: <i>"plot_results.py"</i> with the command as below: <br>
plot_results.py --initial_poses optimized_nodes.txt --optimized_poses optimized_edges.txt
<br>
If you need any futher information or discussion, please send me an email: vanhuong.robotics@gmail.com </br>

