<p align="center">
  <img src="https://www.svgrepo.com/show/263197/magnifying-glass-search.svg" width="100" alt="project-logo">
</p>
<p align="center">
    <h1 align="center">CVLP Cluedo</h1>
</p>
<p align="center">
    <em><code>CVLP (Computer Vision Localisation Project) - Python computer vision simulation project using Gazebo and ROS. The robot must enter the correct room, find the cluedo character card on one of the walls and identify which character it is.</code></em>
</p>

<br><!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary><br>

- [ Overview](#-overview)
- [ Files](#files)
- [ Repository Structure](#-repository-structure)
- [ Modules](#-modules)
- [ Getting Started](#-getting-started)
  - [ Installation](#-installation)
  - [ Usage](#-usage)
- [ Contributing](#-contributing)
</details>
<hr>

##  Overview

<code> This README provides comprehensive instructions to run the developed code for the COMP3631 group project. The objective is to control a simulated Turtlebot in a Gazebo environment to identify a Cluedo character in the "green room" of a two-room setup. The project involves elements of robotics simulation, vision processing, and autonomous navigation using ROS (Robot Operating System) and Gazebo.

For further details, refer to the project descriptions provided in the provided documents `ExampleWorld.pdf`, `ProjectDescription.pdf` and `README.txt`.</code>

---

##  Files

* **cluedo_images/**: Contains images of Cluedo characters for identification.
* **example/**: Example files including `input_points.yaml` for setting initial parameters.
* **src/**: Source directory containing all Python scripts necessary for the simulation.
* **README.txt**: Basic project and run information (expanded in this document).

---

##  Repository Structure

```sh
└── cvlp_cluedo/
    ├── README.md
    └── project
        ├── .gitignore
        ├── ExampleWorld.docx
        ├── ExampleWorld.pdf
        ├── ProjectDescription.docx
        ├── ProjectDescription.pdf
        ├── README.txt
        ├── cluedo_images
        ├── example
        └── src
```

---

##  Modules

<details closed><summary>All Modules</summary>

<details closed><summary>project</summary>

| File                                                                                          | Summary                         |
| ---                                                                                           | ---                             |
| [README.txt](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/README.txt) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.src.el18ac</summary>

| File                                                                                                                 | Summary                         |
| ---                                                                                                                  | ---                             |
| [room_explorer.py](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/src/el18ac/room_explorer.py) | <code>► INSERT-TEXT-HERE</code> |
| [room_locator.py](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/src/el18ac/room_locator.py)   | <code>► INSERT-TEXT-HERE</code> |
| [colour_search.py](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/src/el18ac/colour_search.py) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.src.el18apsr</summary>

| File                                                                                                                         | Summary                         |
| ---                                                                                                                          | ---                             |
| [find_cluedo_rand.py](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/src/el18apsr/find_cluedo_rand.py) | <code>► INSERT-TEXT-HERE</code> |
| [frame_detection.py](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/src/el18apsr/frame_detection.py)   | <code>► INSERT-TEXT-HERE</code> |
| [find_cluedo.py](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/src/el18apsr/find_cluedo.py)           | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.src.el17m3m</summary>

| File                                                                                                                          | Summary                         |
| ---                                                                                                                           | ---                             |
| [room_finder.py](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/src/el17m3m/room_finder.py)             | <code>► INSERT-TEXT-HERE</code> |
| [img_saver.py](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/src/el17m3m/img_saver.py)                 | <code>► INSERT-TEXT-HERE</code> |
| [turtlebot_control.py](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/src/el17m3m/turtlebot_control.py) | <code>► INSERT-TEXT-HERE</code> |
| [identifier.py](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/src/el17m3m/identifier.py)               | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.example</summary>

| File                                                                                                                                | Summary                         |
| ---                                                                                                                                 | ---                             |
| [testing_world.world](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/testing_world.world)             | <code>► INSERT-TEXT-HERE</code> |
| [input_points.yaml](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/input_points.yaml)                 | <code>► INSERT-TEXT-HERE</code> |
| [example_inputs.yaml](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/example_inputs.yaml)             | <code>► INSERT-TEXT-HERE</code> |
| [testing_world_inputs.yaml](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/testing_world_inputs.yaml) | <code>► INSERT-TEXT-HERE</code> |
| [project.world](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/project.world)                         | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.example.models.cluedo_character</summary>

| File                                                                                                                              | Summary                         |
| ---                                                                                                                               | ---                             |
| [model.sdf](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/cluedo_character/model.sdf)       | <code>► INSERT-TEXT-HERE</code> |
| [model.config](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/cluedo_character/model.config) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.example.models.cluedo_character.materials.scripts</summary>

| File                                                                                                                                                                          | Summary                         |
| ---                                                                                                                                                                           | ---                             |
| [Cluedo_character.material](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/cluedo_character/materials/scripts/Cluedo_character.material) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.example.models.green_image</summary>

| File                                                                                                                         | Summary                         |
| ---                                                                                                                          | ---                             |
| [model.sdf](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/green_image/model.sdf)       | <code>► INSERT-TEXT-HERE</code> |
| [model.config](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/green_image/model.config) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.example.models.green_image.materials.scripts</summary>

| File                                                                                                                                                           | Summary                         |
| ---                                                                                                                                                            | ---                             |
| [green_image.material](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/green_image/materials/scripts/green_image.material) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.example.models.grey_wall_small</summary>

| File                                                                                                                               | Summary                         |
| ---                                                                                                                                | ---                             |
| [model.sdf](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/grey_wall_small/model.sdf)         | <code>► INSERT-TEXT-HERE</code> |
| [model.config](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/grey_wall_small/model.config)   | <code>► INSERT-TEXT-HERE</code> |
| [model-1_4.sdf](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/grey_wall_small/model-1_4.sdf) | <code>► INSERT-TEXT-HERE</code> |
| [model-1_3.sdf](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/grey_wall_small/model-1_3.sdf) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.example.models.grey_wall_small.materials.scripts</summary>

| File                                                                                                                                                           | Summary                         |
| ---                                                                                                                                                            | ---                             |
| [grey_wall.material](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/grey_wall_small/materials/scripts/grey_wall.material) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.example.models.red_image</summary>

| File                                                                                                                       | Summary                         |
| ---                                                                                                                        | ---                             |
| [model.sdf](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/red_image/model.sdf)       | <code>► INSERT-TEXT-HERE</code> |
| [model.config](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/red_image/model.config) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.example.models.red_image.materials.scripts</summary>

| File                                                                                                                                                     | Summary                         |
| ---                                                                                                                                                      | ---                             |
| [red_image.material](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/red_image/materials/scripts/red_image.material) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.example.models.red_image.red_image</summary>

| File                                                                                                                                 | Summary                         |
| ---                                                                                                                                  | ---                             |
| [model.sdf](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/red_image/red_image/model.sdf)       | <code>► INSERT-TEXT-HERE</code> |
| [model.config](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/red_image/red_image/model.config) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.example.models.red_image.red_image.materials.scripts</summary>

| File                                                                                                                                                               | Summary                         |
| ---                                                                                                                                                                | ---                             |
| [red_image.material](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/models/red_image/red_image/materials/scripts/red_image.material) | <code>► INSERT-TEXT-HERE</code> |

</details>

<details closed><summary>project.example.map</summary>

| File                                                                                                                  | Summary                         |
| ---                                                                                                                   | ---                             |
| [test_map.yaml](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/map/test_map.yaml)       | <code>► INSERT-TEXT-HERE</code> |
| [project_map.yaml](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/master/project/example/map/project_map.yaml) | <code>► INSERT-TEXT-HERE</code> |

</details>

</details>

---

##  Getting Started

**Prerequisites**

* **ROS**: Robot Operating System, including rospy for Python.
* **Gazebo**: Simulation environment integrated with ROS.
* **OpenCV**: For image processing tasks.
* **Python 2.7**: Required version for compatibility with ROS Kinetic/Kinetic+.
* **turtlebot_gazebo**: ROS package for simulating Turtlebot in Gazebo.
* **turtlebot_rviz_launchers**: To launch RViz for visualizing Turtlebot navigation.
* **turtlebot_teleop**: Package for manual control of Turtlebot via keyboard.

###  Installation

<h4>From <code>source</code></h4>

> 1. Clone the cvlp_cluedo repository:
>
> ```console
> $ git clone https://github.com/Alexpascual28/cvlp_cluedo.git
> ```
>
> 2. Change to the project directory:
> ```console
> $ cd cvlp_cluedo
> ```
>
> 3. Before running the project, ensure all dependencies are installed:
> ```console
> $ sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-rviz-launchers ros-kinetic-turtlebot-teleop
pip install opencv-python
> ```

###  Usage

**Configuration Files**

Modify `input_points.yaml` in the `example/` directory to adjust the initial points as per your specific test setup.

**Python Scripts Execution Order**

1. `find_cluedo_rand.py`: Randomizes the search within the simulation environment.

> ```console
> python src/el18apsr/find_cluedo_rand.py
> ```

2. `identifier.py`: Processes image data to identify the Cluedo character.

> ```console
> python src/el17m3m/identifier.py
> ```

3. `room_finder.py`: Detects and navigates to the room with the green circle.

> ```console
> python src/el17m3m/room_finder.py
> ```

4. `turtlebot_control.py`: Controls the Turtlebot's movement within the environment. This script should be run last to commence the demo.

> ```console
> python src/el17m3m/turtlebot_control.py
> ```

**Running the Simulation**

To start the simulation, follow these steps:

1. Launch the Gazebo world tailored for Turtlebot:

> ```console
> roslaunch turtlebot_gazebo turtlebot_world.launch
> ```

2. Initialize the map and localization:

> ```console
> roslaunch simulated_localisation.launch map_file:=<path_to_map_file>
> ```

3. Start RViz for visualization:

> ```console
> roslaunch turtlebot_rviz_launchers view_navigation.launch
> ```

4. Localize the robot using "2D pose estimate" in RViz.

5. Use turtlebot_teleop to position the robot:

> ```console
> roslaunch turtlebot_teleop keyboard_teleop.launch
> ```

6. Terminate the teleop node and execute your scripts as detailed above.

**Expected Outputs**

* **green_circle.png**: Image of the green circle detected.
* **cluedo_character.png**: Image of the Cluedo character.
* **cluedo_character.txt**: Text file with the name of the identified Cluedo character.

---

By following the steps and configurations detailed in this guide, your Turtlebot should be equipped to autonomously navigate the simulation environment, identify the correct room, and recognize the Cluedo character within the constraints and requirements of the COMP3631 project.

For further details, refer to the project descriptions provided in the provided documents `ExampleWorld.pdf`, `ProjectDescription.pdf` and `README.txt`.

---​

##  Contributing

Contributions are welcome! Here are several ways you can contribute:

- **[Report Issues](https://github.com/Alexpascual28/cvlp_cluedo.git/issues)**: Submit bugs found or log feature requests for the `cvlp_cluedo` project.
- **[Submit Pull Requests](https://github.com/Alexpascual28/cvlp_cluedo.git/blob/main/CONTRIBUTING.md)**: Review open PRs, and submit your own PRs.
- **[Join the Discussions](https://github.com/Alexpascual28/cvlp_cluedo.git/discussions)**: Share your insights, provide feedback, or ask questions.

<details closed>
<summary>Contributing Guidelines</summary>

1. **Fork the Repository**: Start by forking the project repository to your github account.
2. **Clone Locally**: Clone the forked repository to your local machine using a git client.
   ```sh
   git clone https://github.com/Alexpascual28/cvlp_cluedo.git
   ```
3. **Create a New Branch**: Always work on a new branch, giving it a descriptive name.
   ```sh
   git checkout -b new-feature-x
   ```
4. **Make Your Changes**: Develop and test your changes locally.
5. **Commit Your Changes**: Commit with a clear message describing your updates.
   ```sh
   git commit -m 'Implemented new feature x.'
   ```
6. **Push to github**: Push the changes to your forked repository.
   ```sh
   git push origin new-feature-x
   ```
7. **Submit a Pull Request**: Create a PR against the original project repository. Clearly describe the changes and their motivations.
8. **Review**: Once your PR is reviewed and approved, it will be merged into the main branch. Congratulations on your contribution!
</details>

<details closed>
<summary>Contributor Graph</summary>
<br>
<p align="center">
   <a href="https://github.com{/Alexpascual28/cvlp_cluedo.git/}graphs/contributors">
      <img src="https://contrib.rocks/image?repo=Alexpascual28/cvlp_cluedo.git">
   </a>
</p>
</details>

---