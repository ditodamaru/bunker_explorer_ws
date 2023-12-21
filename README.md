<a name="readme-top"></a>

# Autonomous Driving Framework for Orchard Environment ROS-based
Repository ini tentang autonomous driving framework yang telah diuji pada simulation dan real field test berbasis ROS 1 Melodic. Telah ditest pada Ubuntu 18.04 

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>


<!-- ABOUT THE PROJECT -->
## About The Project

 Autonomous Driving Framework for Orchard Environment ROS-based

<p align="right">(<a href="#readme-top">back to top</a>)</p>



### Built With

 

* [![ROS 1 Melodic]
* [![Linux Ubuntu 18.04]]
* {![RTABMAP]]

<p align="right">(<a href="#readme-top">back to top</a>)</p>




<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* Install Kubernetes
  ```sh

  ```
* Install Prometheus
  ```sh
  
  ```
* Install module random
  ```sh
  pip install random2
  ```
  



### Installation

_Below is an example of how you can instruct your audience on installing and setting up your app. This template doesn't rely on any external dependencies or services._

1. Install miniforge on raspberry pi or your computer [https://github.com/conda-forge/miniforge](https://github.com/conda-forge/miniforge)
2. Go to desired workspace folder
   ```sh 
   cd ~/github/aquaponic_ws/source/greenhouse/
   ```
3. create new conda environment using miniforge
   For pc : 
   ```sh
   conda create -n env_pc_py38 python=3.8
   ```
   For raspi
   ```sh
   conda create -n env_raspi_py38 python=3.8
   ```
4. create new folder to indicate the env name
   ```sh
   mkdir env_pc_py38
   ```
5. Activate newly conda environment
   ```sh
   conda activate env_pc_py38
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## 1. create new directory
### mkdir github
### cd github
### mkdir -p bunker_explorer_ws/src
### cd bunker_explorer_ws
### catkin_make
## 2. git clone this repository to your directory
### catkin_make_isolated
## 3. install dependencies
### sudo apt install ros-melodic-rtcm-msgs 
### install bunker_description package from AgileX github repository
## 4. add source directory to ROS PATH environment


