# smoking-detection-mobile-surveillance-robot
The final year project by Chia Jing Hui, Chong Jia Mun, Goh Yan Cheng, students from AI Department, University of Malaya

**Environment:**
<ul>
<li>Ubuntu 18.04</li>
<li>ROS Melodic</li>
<li>Python 3.8</li>
</ul>

**Steps to Run:**

<ol>
<b>In a terminal:</b>
<li>Navigate to the bash script location</li>

$ `roscd yolov5_ros/scripts`
<li>Run the bash script to bring up all the program</li>

$ `./bringup.sh`

<b>In a new terminal:</b>
<li>Activate the Conda environment</li>

$ `conda activate ycpytorch`
<li>Navigate to the bash script location</li>

(ycpytorch) $ `roscd yolov5_ros/scripts`
<li>Run the bash script to launch the programs</li>

(ycpytorch) $ `./run.sh`
</ol>

**NOTE:**
<ul>
<li>Please remember to change the image paths before executing the code</li>

<li>Please ensure that all the files in the scripts folder are executable  </li>

`chmod +x <filename>.py`
</ul>
