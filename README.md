师兄们完成的ssvo+闭环,这里是我为了自己理解所添加的注释.

许可师兄的原版 [ssvo](https://github.com/kokerf/ssvo)
姜浩师兄和邓毅师兄添加了闭环部分 [ssvo](https://github.com/JHzss/ssvo)
原 README.MD 文档如下

-----

# ssvo

Semi-direct sparse odometry

video of running in [rpg_urban dataset](https://www.youtube.com/watch?v=2AnIj_QFtow), and [live video](https://www.youtube.com/watch?v=ISGDrrDcUB0).

### 1. Prerequisites

#### 1.1 [OpenCV](http://opencv.org)
OpenCV 3.1.0 is used in this code.

#### 1.2 [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
```shell
sudo apt-get install libeigen3-dev
```

#### 1.3 [Sophus](https://github.com/strasdat/Sophus)
This code use the template implement of Sophus. Recommend the latest released version [v1.0.0](https://github.com/strasdat/Sophus/tree/v1.0.0)

#### 1.4 [glog](https://github.com/google/glog)
This code use glog for logging.
```shell
sudo apt-get install libgoogle-glog-dev
```

#### 1.5 [Ceres](http://ceres-solver.org/installation.html)
Use Ceres-Slover to slove bundle adjustment. Please follow the [installation page](http://ceres-solver.org/installation.html#section-customizing) to install.

#### 1.6 [Pangolin](https://github.com/stevenlovegrove/Pangolin)
This code use Pangolin to display the map reconstructed. When install, just follow the [README.md](https://github.com/stevenlovegrove/Pangolin/blob/master/README.md) file.

#### 1.7 [DBow3](https://github.com/kokerf/DBow3)
After build and install, copy the file `FindDBoW3.cmake` to the directory `cmake_modules`

### 2. Usages & Evalution
the Euroc Dataset is used for evalution. In the project directory, run the demo by the following commond
```shell
./bin/monoVO_euroc ./config/euroc.yaml ./calib/euroc_cam0.yaml dataset_path
```
**note that** the `dataset_path` is the directory contaion `body.yaml`

finally, there will be a `trajectory.txt` saved, and you can use the [evo](https://github.com/MichaelGrupp/evo) to evaluate.
