# Drone-Created Art: from PNG to X, Y, Z

Project for AE 483: Autonomous Systems Lab

Created by Caleb Carrigan, Anna Hylbert, Eric Monson, and Matt Taylor

## Code Structure

Our repository contains the following structure:


```analysis/```: Finalized Jupyter notebooks and data.

- ```results/```: Contains logs and videos from test flights of our drone.
- ```moi.ipynb```: Lab03 modified for our final project. Calculates moment of inertia in the x, y, and z directions.
- ```force_and_moment_parameter.ipynb```: Lab04 modified for our final project. Calculates $k_f$ and $k_m$.
- ```Observer_Model.ipynb```: Lab07 modified for our final project.
- ```Observer_Offline_Implementation.ipynb```: Lab08 modified for our final project.
- ```Observer_Online_Implementation.ipynb```: Lab09 modified for our final project.


```client/```: Contains crazyflie client code for flying the drone in Python.

- ```data/```: Contains images used for edge detection.
- ```edge_detector.py```: Contains classes and functions pertaining to edge detection. Canny edge detection, DFS traversal, etc.
- ```flight.py```: Flight code used to draw images. 3 step pipeline to go from image -> edge mask -> move commands -> drone flight. Uses command line arguments to input an image along with hyper-parameters of standard deviation, image height, width, etc. 


```firmware/```: Finalized versions of modified crazyflie firmware code modified for the use of the lighthouse deck.

- ```controller_ae483.h```: An additional function is added called `ae483UpdateWithLighthouse` which tracks the lighthouse position data.
- ```controller_ae483.c```: Contains a custom controller and observer implementation.
- ```lighthouse_position_est.c```: Default firmware is modified to grab x, y, and z position calculated using the crossing beams method of the lighthouse positioning system.


## Environment Setup

In addition to the virtual environment created for `AE 483`, two additional libraries must be installed for the edge detector.  These can be installed with the following commands.

```
conda install -c conda-forge opencv
conda install -c anaconda scikit-image
```

## Additional Information

### Edge Detection Examples

#### Simple Circle

![Circle Edge](https://user-images.githubusercontent.com/60635839/205718195-9ba7f97a-6c8a-4e2d-a083-b9ce3576ed5d.png)

##### Custom Path Finding Algorithm
![circle](https://user-images.githubusercontent.com/60635839/205718618-ce949293-25b9-44db-b8ef-e3e3c2efc36e.gif)

#### Hexagon

![Hexagon Edge](https://user-images.githubusercontent.com/60635839/205718227-a0515f06-d38e-48af-b2c0-53e8b0037d5f.png)

##### Custom Path Finding Algorithm
![hexagon](https://user-images.githubusercontent.com/60635839/205718598-8e425c3d-336d-44a0-92d6-8ce9a1dbdb6f.gif)

#### Google Logo

![Google Edge](https://user-images.githubusercontent.com/60635839/205718163-b6124139-0873-447a-a624-d01f2e18532c.png)

##### Custom Path Finding Algorithm

![google](https://user-images.githubusercontent.com/60635839/205718544-fc666cd8-1dac-46e6-af81-5c2d2a3366c3.gif)

### Resources

Jain, R., and et al., Machine Vision, McGraw-Hill, 1995, Chap. 5: Edge Detection, pp. 140–180.
