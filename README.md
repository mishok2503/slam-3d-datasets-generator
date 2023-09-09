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

4. `LIDAR_MAX_DEPTH`:
   - Indicates the maximum distance that the laser can measure, if the distance is greater than the point type will be `MAX`

5. `mapGenerator`:
   - Variable responsible for the map generation model.
   - Example: `TMazeMapGenerator(MAP_SIZE)` or `TCubeMapGenerator(MAP_SIZE)`
6. `lidar`:
    - Variable responsible for the type of lidar.
    - Example: `TSimpleLidar(LIDAR_POINTS_COUNT, LIDAR_MAX_DEPTH)` or `TFibonacciLidar(LIDAR_POINTS_COUNT, LIDAR_MAX_DEPTH)`

7. `robotBuilder`:
    - A pointer to the robot builder object responsible for constructing the robot with the specified lidar.
    - Example: `TRobotBuilder(std::move(lidar))`

8. `errorModel`:
    - A pointer to the error model object representing the error applied to lidar measurements and odometry.
    - Examples:
      - ```C++
        std::unique_ptr<IErrorModel> errorModel{
            new TSphericalUniformErrorModel(
                /*** Odometry ***/
                // the boundary of the uniform distribution for the robot position
                // that is, a random variable from a uniform distribution [-0.02; 0.02] is added to the robot's odometry for each coordinate
                0.02,
                // same for orientation
                0.05,
                /*** Lidar ***/
                // functions return boundaries for uniform distribution
                // for example, if the distance (radius) to the point is 10,
                // then at the resulting point the radius will be in [10 - 0.2; 10 + 0.2]
                [](float r) { return r / 50; },
                // in this function, the error does not depend on the angle and at the resulting point,
                // the value of the theta angle will differ from the original by +-0.01
                [](float theta) { return 0.01; },
                [](float phi) { return 0.02; }
        )};
        ```
      - ```C++
        std::unique_ptr<TErrorModel2D<false>> errorModel{
            new TErrorModel2D<false>( // the second parameter is whether the lidar error is two-dimensional
                0.02, // variance for position error
                0.001, // variance for orientation error
                0.005 // variance for lidar point position
        )};
        // function for setting the quality distribution
        // with a probability of 80%, the quality will be 1, otherwise 0.1
        errorModel->SetQualityFunction([](float x) {
            return x < 0.8 ? 1.f : 0.1f;
        });
        // the coefficient indicates how much the quality affects the error
        // the formula by which the variance is recalculated: variance * (1 + (1 - quality) * QualityCoef)
        // in this example, if the measurement quality of the point is 0.1, then the error variance will be 28 times greater
        errorModel->SetQualityCoef(30);
        ```

9. `simulator`:
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

Note that the robot **firstly rotates** and writes the odometry of the rotation
and **only then moves** and writes the odometry of the position relative to the new orientation.

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
