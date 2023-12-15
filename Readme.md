# Autonomous Vehicle Navigation System

## Overview
This repository contains Python scripts to simulate an autonomous vehicle navigation system using the Carla simulator. The system comprises several modules to plan routes, control vehicle speed and steering, and execute autonomous navigation in the Carla simulation environment.

## Requirements
- Python 3.8.0
- Carla Simulator (version 0.9.14)
- NumPy
- NetworkX
- Carla Python API (provided by the Carla Simulator)

## Modules

### `global_route_planner.py`
- **Description**: Provides a high-level route planning algorithm.
- **Classes**: `GlobalRoutePlanner`
- **Dependencies**: `carla`, `numpy`, `networkx`

### `lateral_controller.py`
- **Description**: Implements the Pure Pursuit Controller for lateral vehicle control.
- **Classes**: `PurePursuitController`
- **Dependencies**: `math`, `numpy`

### `longitudinal_controller.py`
- **Description**: Contains the PID Longitudinal Controller for vehicle speed control.
- **Classes**: `PIDLongitudinalController`
- **Dependencies**: `numpy`, `math`, `collections`

### `utils.py`
- **Description**: Provides utility functions used across modules.
- **Functions**: `find_dist_veh`, `get_speed`, `vector`, `control_signal`
- **Dependencies**: `math`, `numpy`, `carla`

### `main.py`
- **Description**: Executes the autonomous navigation system using Carla.
- **Dependencies**: All modules mentioned above and `carla`

## Usage
1. Ensure the Carla Simulator is installed and configured properly.
2. Run `main.py` to start the autonomous navigation simulation.

## How It Works
- `global_route_planner.py` generates a high-level route plan using the Carla map.
- `lateral_controller.py` implements the Pure Pursuit algorithm for lateral control.
- `longitudinal_controller.py` contains a PID controller for longitudinal speed control.
- `utils.py` provides helper functions for distance calculation, speed retrieval, etc.
- `main.py` orchestrates the execution, spawns the vehicle, plans the route, and controls the vehicle's movement.

## Simulation Process
1. The system plans a route using the Global Route Planner.
2. A vehicle is spawned at a random start point and follows the planned route.
3. The vehicle uses PID controllers for speed and Pure Pursuit for steering to navigate.
4. The simulation ends when the vehicle reaches the last waypoint.


