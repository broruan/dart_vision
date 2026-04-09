# （迈德威视）相机类及其ROS2节点类设计草稿

## 一些原则

首先，充分利用迈德威视的SDK来做开发。  
在此基础上，希望能在运行日志中监测记录相机硬件的状态。  
为此在程序中定义一个虚拟的迈德威视相机，在其中记录对应相机硬件的所有可以得到的属性。  
  
另一方面，充分利用ROS2的特性，图片数据必须零拷贝。  
需要共享给多个节点的数据，比如相机内参，使用ROS2对应的消息类型在话题里发布。  
尽可能使用参数服务进行运行时参数调试。  

### 奇思

让相机节点双继承，即是一个ROS2节点，又是一个Mindvision相机

## 相机节点的功能所需

1. 视频录制功能
2. 中间数据录制功能（录ros包）
3. 支持多相机不掉帧
4. 占用优化（程序性能开销优化）
5. 动态调参，运行时调参（参数列表是要确定的）

## 一些关于

### 关于命令行输出颜色

用的变色方法可行，只是颜色不怎么多，比较原始  
上交开源的仓库里好像有用一个多彩的第三方包  

## 参考资料

参考南航的相机节点：  
[nuaa-rm/md_camera](https://github.com/nuaa-rm/md_camera "南航相机节点")  
太可惜了，ros的部分参考不了，南航这个两年前开源的包用的是ros1。  
参考ros2社区贡献的零拷贝感知包
[](https://github.com/ZhenshengLee/ros2_shm_msgs)
参考

参数相关  
这个是总的文档，里面有连接到各个parameter使用参数细节文档。  
[ROS2参数较详细的介绍，主要是里面的 Parameter callbacks 章节](https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html)  
[ROS2参数命令行工具使用](https://docs.ros.org/en/humble/How-To-Guides/Using-ros2-param.html)  
使用on_parameter_event在某些参数发生改变的时候，做出所需的行动，比如说修改类中公共数据的值（修改的时候可能需要加锁）。  
参考了，ROS2官方提供的一些demo:

1. even_parameters_node 如何拒绝不符合范围的参数
2. list_parameters 列举参数的写法，有同步阻塞和异步非阻塞两种
3. parameter_blackboard 允许未声明的参数自动声明
4. parameter_events 这是最重要的要用的（可能），每太看懂这三个程序，里面有异步编程

rosbag2相关
[](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)
[](https://docs.ros.org/en/humble/Tutorials/Advanced/Recording-A-Bag-From-Your-Own-Node-CPP.html)  
[](https://docs.ros.org/en/humble/Tutorials/Advanced/Reading-From-A-Bag-File-CPP.html)  
[Paramters Should be Boring](https://vimeo.com/879001499)  

多线程执行器相关
[](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html)  
