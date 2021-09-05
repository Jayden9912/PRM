# Probabilistic Roadmap and Path Shortcutting

This repository is created for the submission to the exercises given in this webpage.\
Webpage   : http://www.osrobotics.org/osr/planning/path_planning.html \
Exercises : "Solving a 2D motion planning problem by PRM" and "Post-processing a 2D path" \
For more information, please refer to http://www.osrobotics.org/osr/

## Usage
1. Create a new file
```
mkdir -p ~/src && cd ~/src
```
2. Clone this github repository.
```
git clone https://github.com/Jayden9912/PRM.git
```
3. Edit the permission of the shell file.
```
sudo chmod +x run.sh
```
4. Run the code
```
./run.sh
```
## What can it do?
Given static obstacle, initial position and target position, probabilistic roadmap will be created based on the conditions given (with pathshorcut enabled).\
![This is an image](https://github.com/Jayden9912/PRM/blob/main/pic/allPlot.png)
There are **few options** that can be changed in run.sh. Particularly, the option of **edgePlot** and **pointPlot** should be set to False to reduce the runtime.\
With edgePlot, pointPlot and **shortcut set to False**:
![This is an image](https://github.com/Jayden9912/PRM/blob/main/pic/no_shortcut.png)
With edgePlot, pointPlot set to False, **shortcut set to True**:
![This is an image](https://github.com/Jayden9912/PRM/blob/main/pic/shortcut1.png)
