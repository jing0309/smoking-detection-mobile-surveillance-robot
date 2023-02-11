# smoking-detection-mobile-surveillance-robot
The final year project by Chia Jing Hui, Chong Jia Mun, Goh Yan Cheng, students from AI Department, University of Malaya

**Environment:**
<ul>
<li>Ubuntu 18.04</li>
<li>ROS Melodic</li>
<li>Python 3.8.15</li>
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

### Ways to create virtual environment:
- Download the corresponding installation package [Anaconda][1]
- Install Anaconda
```python
# change the path according to your environment
bash ~/Downloads/Anaconda3-2022.10-Linux-x86_64.sh
```
- Edit .bashrc file
```python
# to open and edit the file
sudo nano ~/.bashrc
```
```python
# add this line to the end of the file
export PATH=/home/mustar/anaconda3/bin:$PATH
```
- `Ctrl+O` , `Enter` , `Ctrl+x` (save and exit)
- preserve your entire shell environment 
```python
source ~/. bashrc
```
- Create Environment
```python
conda create --name <environment_name> python=3.8
```

[1]: https://www.anaconda.com/products/distribution#linux

### Installing Libraries/Packages Needed for the Image Processing module:
- Activate virtual environment 
```python
conda activate <environment_name>
```
- Install Libraries/Packages (Make sure you're in the [yolov5_ros][2] directory)
```python
pip install -r requirements.txt
```
[2]:https://github.com/jing0309/smoking-detection-mobile-surveillance-robot/tree/main/yolov5_ros
### Important parameters to note in the [launch](https://github.com/jing0309/smoking-detection-mobile-surveillance-robot/tree/main/yolov5_ros/launch) folder of `yolov5_ros`:

- `weight_path`
Path to the trained weights

- `image_topic`
Robot camera index

- `conf`
Confidence level

- `image_path`
Path to the image

**NOTE:**
<ul>
<li>Please remember to change the image paths before executing the code</li>

<li>Please ensure that all the files in the scripts folder are executable  </li>

`chmod +x <filename>.py`
</ul>
