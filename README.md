# Drone-Created Art: from PNG to X, Y, Z

Created by Caleb Carrigan, Anna Hylbert, Eric Monson, and Matt Taylor

## Code Structure

Our repository contains the following structure:


```firmware/```: Finalized versions of modified crazyflie firmware code.

- ```file1.py```: Add description.

- ```file2.py```: Add description.

```client/```: Contains crazyflie client code for flying the drone in Python.

- ```data/```: Contains images used for edge detection.

- ```file1.py```: Add description.

- ```file2.py```: Add description.


```analysis/```: Finalized Jupyter notebooks and data.

- ```results/```: Contains logs and videos from test flights of our drone.

- ```file1.ipynb```: Add description.

- ```file2.ipynb```: Add description.

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
