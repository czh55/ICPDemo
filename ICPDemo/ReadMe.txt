========================================================================
    控制台应用程序：ICPDemo 项目概述
========================================================================
这四个文件全部由main，可以单独运行，所以在vs中进行配置，只选择其中一个！

ICPDemo.cpp：就是简单的使用ICP，读入单个点云，旋转后于原点云进行icp配准,最后显示出来
ICPDemo_rotation.cpp：对矩阵旋转的测试，并在单通道下显示出来。读入两个点云(读入时两个点云时大范围重合的，点云是我自己切的)，只旋转其中一个，看这两个点云的相对位置。
ICPDemo_segmentation.cpp：主要文件.依然是是用ICP算法，单独写函数：print4x4Matrix() keyboardEventOccurred() savePointCloudFile()合并之后保存点云
PointAll2One.cpp：两个点云进行合并，保存
SAC_ICP.cpp：先SAC，在ICP配准
RegistrationUsingPlane.cpp：基于平面的配准，有自己的数据集：dataForPlane

参考博客ICPDemo.cpp：
https://blog.csdn.net/zmdsjtu/article/details/79871165
参考博客PointAll2One.cpp：
https://blog.csdn.net/sunboyiris/article/details/72636809
https://blog.csdn.net/baidu_26408419/article/details/72630664?locationNum=8&fps=1


