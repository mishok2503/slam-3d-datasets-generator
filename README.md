# 3D SLAM Datasets Generator

This repository contains the source code for a dataset generator used for testing SLAM algorithms. This generator allows you to generate 3D data in json format, and also allows you to configure some data parameters, such as the map model, lidar type and error model.

## Usage

To use the dataset generator, follow these steps:

#### Clone the repository:

```bash
git clone https://github.com/mishok2503/slam-3d-datasets-generator.git
cd slam-3d-datasets-generator
```

#### Build

```bash
mkdir -p build && cd build
cmake ..
make
cp ../visualize.py .
```
   
#### Run
   
```bash
./generator [output-file]
```

Default `output-file` name is `"result.json"`.

You will get two files:
1. `gt_"output-file"` - contains the result of the generator without errors (ground_truth).
2. `"output-file"` - same, but with errors.

## Settings

To configure the generator, you need to change the `main.cpp` file, which contains variables that are responsible for the corresponding parameters. After that, the project needs to be recompiled and launched again.

## Code Documentation

This part will provide an overview of the variables used in the code.

### Variables:

1. `MAP_SIZE`:
    - Represents the size of the map in three dimensions.
    - Format: {x, y, z}
    - Example: {7, 7, 7}

2. `STEPS_COUNT`:
    - Specifies the number of steps the robot will take in the simulation.
    - Example: `800`

3. `LIDAR_POINTS_COUNT`:
    - Indicates the number of lidar points the robot will take in each step.
    - Example: `1500`

4. `mapGenerator`:
    - Variable responsible for the map generation model.
    - Example: `TMazeMapGenerator(MAP_SIZE)` or `TCubeMapGenerator(MAP_SIZE)`

5. `lidar`:
    - Variable responsible for the type of lidar.
    - Example: `TSimpleLidar(LIDAR_POINTS_COUNT, /*max depth*/ 15)` or `TFibonacciLidar(LIDAR_POINTS_COUNT, 15)`

6. `robotBuilder`:
    - A pointer to the robot builder object responsible for constructing the robot with the specified lidar.
    - Example: `TRobotBuilder(std::move(lidar))`

7. `errorModel`:
    - A pointer to the error model object representing the error applied to lidar measurements.
    - Example: `TUniformErrorModel(0.02, 0.05, errorFromRadius, errorFromTheta, errorFromPhi)`
      - The first two parameters are responsible for the error boundaries for odometry - position and rotation, respectively.
      - the remaining parameters of this function, which return the boundary of a uniform distribution over the spherical coordinates of the point - radius, theta and phi, respectively.

8. `simulator`:
    - An instance of the Simulator class that orchestrates the simulation process using the provided map, robot, and error model objects.


## Output format

Data present in the ground_truth file only:
```json
{
  "map": {
    "size": "array of coordinates",
    "data": "3d array describing the occupancy of cells",
    "robot_start_position": {
      "position": "array of coordinates",
      "euler_angles": "array of coordinates"
    },
    "error_model": {
      "type": "uniform_error",
      "rotation_error": 0.02,
      "position_error": 0.05
    }
  }
}
```

The data contained in both files:

```json
{
  "data": {
    "steps_count": 500,
    "lidar_points_count": 3000,
    "measurements": [
      {
        "lidar_data": [
          {
            "type": "unknown/max/point",
            "coordinates": "array of coordinates"
          }
        ],
        "odometry": {
          "position": "array of coordinates delta",
          "euler_angles": "array of coordinates delta"
        }
      }
    ]
  }
}
```

## Test the result

Install open3d library:
```bash
   pip install open3d
```

Run the visualization:
```bash
  python vizualize.py <output-file>
```


## How it works

To generate data, the following steps are performed:

1. Creating a map: Identify the environment by generating a map that will serve as a physical obstacle for the robot.

2. Robot Placement: Randomly place the robot in an unoccupied cell on the map.

3. Lidar emulation: Emulate the real behavior of the lidar by emitting rays in all directions and obtaining appropriate distance measurements or information about rays that did not reach any obstacles.

4. Robot Movement: Move the robot considering its forward direction and speed. Move the robot directly along its trajectory if there are no obstacles ahead. If the robot is too close to the wall, rotate it arbitrarily, making sure that it moves away from the wall in the next cycle.

5. Repeat and generate data: Repeat the steps described above until the desired amount of data is generated.
