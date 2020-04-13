# Implementation of A* algorithm on a differential drive (non-holonomic) TurtleBot robot

## Authors
- Pruthvi Sanghavi (UID: 116951005)
- Achal Vyas

## Dependencies
- numpy ```pip install numpy```
- matplotlib ```pip install matplotlib```

## Run Instructions
### From Repository
Open the terminal 
```
cd <desired directory>
git clone https://github.com/Achalpvyas/Project3_phase_3.git
cd Project3_phase_3
python3 planner.py
```
- Enter the coordinates of the start point ```sx = -4.5```,```sy = -4.5```,``` theta_s = 30```, the coordinates of the goal point ```gx = 4.5```,``` gy = 4.5```, the values of rotation of the wheels ```rotationL = 10```, ```rotationR = 15``` and ```clearance = 2```.

### From Compressed zip package
- Download the zip folder.
- Enter the desired directory.
- Unzip the folder.
- Open a terminal,
```
cd proj3_7_simulationSoftware
python3 planner.py
```
## Results
![Results](https://github.com/Achalpvyas/Project3/blob/master/Phase_3/result.gif)
