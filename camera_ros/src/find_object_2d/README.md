# find-object

<table>
    <tbody>
        <tr>
           <td>Linux</td>
           <td><a href="https://github.com/introlab/find-object/actions/workflows/cmake.yml"><img src="https://github.com/introlab/find-object/actions/workflows/cmake.yml/badge.svg" alt="Build Status"/> <br> <a href="https://github.com/introlab/find-object/actions/workflows/ros1.yml"><img src="https://github.com/introlab/find-object/actions/workflows/ros1.yml/badge.svg" alt="Build Status"/> <br> <a href="https://github.com/introlab/find-object/actions/workflows/ros2.yml"><img src="https://github.com/introlab/find-object/actions/workflows/ros2.yml/badge.svg" alt="Build Status"/>
           </td>
        </tr>
        <tr>
           <td>Windows</td>
           <td><a href="https://ci.appveyor.com/project/matlabbe/find-object/branch/master"><img src="https://ci.appveyor.com/api/projects/status/hn51r6p5c0peqctb/branch/master?svg=true" alt="Build Status"/>
           </td>
        </tr>
     </tbody>
  </table>

## Standalone
Find-Object project, visit the [home page](http://introlab.github.io/find-object/) for more information.

## ROS1

### Install

Binaries:
```bash
sudo apt-get install ros-$ROS_DISTRO-find-object-2d
```

Source:

 * To include `xfeatures2d` and/or `nonfree` modules of OpenCV, to avoid conflicts with `cv_bridge`, build same OpenCV version that is used by `cv_bridge`. Install it in `/usr/local` (default).

```bash
cd ~/catkin_ws
git clone https://github.com/introlab/find-object.git src/find_object_2d
catkin_make
```

### Run
```bash
