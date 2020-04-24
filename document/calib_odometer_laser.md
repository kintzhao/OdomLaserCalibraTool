

Simultaneous calibration of odometry and sensor parameters for mobile robots

[toc]



## 1. 问题描述：

   机器人坐标系看作odom坐标系， 传感器坐标系是激光坐标系  。 

   ![image-20200422183920804](calib_odometer_laser.assets/image-20200422183920804.png) 

   物理量表示含义

​      ![image-20200422183948089](calib_odometer_laser.assets/image-20200422183948089.png)



## 2. 外参标定描述：

   标定参数为左右轮半径 $r_l$ $r_r$ ，轮间距$b$， 和激光相对轮子的外参（x,y,yaw）

   输入： 左右轮转度（rad/s）      激光里程计数据

   输出： 标定参数 

###  2.1 运动模型： 左右轮线速度转化为基体的线速度与角速度

![image-20200422190307693](calib_odometer_laser.assets/image-20200422190307693.png)

![image-20200422190255556](calib_odometer_laser.assets/image-20200422190255556.png)

![image-20200422190728880](calib_odometer_laser.assets/image-20200422190728880.png)

​										其中， ![image-20200422192108380](calib_odometer_laser.assets/image-20200422192108380.png)	　

### 2.2 SE２ 计算公式：

​	![image-20200422191523834](calib_odometer_laser.assets/image-20200422191523834.png)　　 

​	SE2坐标变换的关系

![image-20200423094543124](calib_odometer_laser.assets/image-20200423094543124.png)

​		![image-20200423095059764](calib_odometer_laser.assets/image-20200423095059764.png)		 



### 2.3  优化方程

![image-20200422192432786](calib_odometer_laser.assets/image-20200422192432786.png)



### 2.4 求解方法：

3种通用求解方案： 

​	1）基于模型的控制理论：连续时间
![image-20200423095813211](calib_odometer_laser.assets/image-20200423095813211.png)

​	2） 静态分析：
​	y离散可观， 确定性的的输入x， 加上足够的约束一起去求解X.

![image-20200423095909838](calib_odometer_laser.assets/image-20200423095909838.png)

​	3) 统计分析：Fisher Information Matrix (FIM)： 线性系统
 	u选择适当的情况下如果FIM中X是满秩的，系统可观

![image-20200423100251993](calib_odometer_laser.assets/image-20200423100251993.png)

==》 本校正系统选用方法二的静态分析（离散的，非线性系统）

前提约束条件：

​	1) 全局参数变量的二义性：  ==》 b>0 来区分

![image-20200423101005194](calib_odometer_laser.assets/image-20200423101005194.png)

​	2） 标定参数可观的条件：  **左右轮独立**， **机器人观察区间内移动需要有旋转**

![image-20200423101127249](calib_odometer_laser.assets/image-20200423101127249.png)

​	3）轨迹速度与轮子转速： 线性独立关系

![image-20200423101948409](calib_odometer_laser.assets/image-20200423101948409.png)



## 3. 标定系统问题

![image-20200423102428429](calib_odometer_laser.assets/image-20200423102428429.png)

![image-20200423102147003](calib_odometer_laser.assets/image-20200423102147003.png)

非凸优化问题 ==》  通常的数值优化问题并不能有效解决。==》 closed form

本标定算法是建立在大前提假设：

![image-20200423103041009](calib_odometer_laser.assets/image-20200423103041009.png)

补充： 假设一不成立就得考虑 ==》covariance inflation 

 ==》 旋转和平移的相互独立性，不相关。==》   =》 系统分解为线性部分和非线性部分

### 3.1  线性估计 $J_{21}$  $J_{22}$： 旋转的一致和可观性

![image-20200423104417595](calib_odometer_laser.assets/image-20200423104417595.png)
$\hat s_{\theta}^{k} ==  s_{\theta}^{k} $   ,且与  $J_{21}$  $J_{22} $ 线性相关

![image-20200423104853352](calib_odometer_laser.assets/image-20200423104853352.png)

其中，![image-20200423104917683](calib_odometer_laser.assets/image-20200423104917683.png)

==>  线性系统 =》 最新二乘问题  连续系统离散化处理  ==》估计 得到参数 $\hat J_{21}$  $ \hat J_{22} $   也就是$-r_L/b , r_R/b$

![image-20200423105043652](calib_odometer_laser.assets/image-20200423105043652.png)

### 3.2 非线性估计其他参数 $b,L_x,L_y, L_{\theta}$   

参差项转化：（不同坐标系下参差表示）
![image-20200423105843468](calib_odometer_laser.assets/image-20200423105843468.png)

优化问题转化为：
![image-20200423110329786](calib_odometer_laser.assets/image-20200423110329786.png)

问题求解：分解

![image-20200423110416574](calib_odometer_laser.assets/image-20200423110416574.png)

![image-20200423110459220](calib_odometer_laser.assets/image-20200423110459220.png)

​	



1）里程计运动平移部分的表示为：

![image-20200423110558251](calib_odometer_laser.assets/image-20200423110558251.png)

​			其中， 

![image-20200423110627503](calib_odometer_laser.assets/image-20200423110627503.png)

**平移运动简化为：** ![image-20200423110833191](calib_odometer_laser.assets/image-20200423110833191.png)

​		其中：
![image-20200423110912777](calib_odometer_laser.assets/image-20200423110912777.png)

​		公式18,19中的 $r_{\theta}^k$  参考公式13的离散化

​	2） 系统转化为二次型系统问题：
​		参数：

![image-20200423111424865](calib_odometer_laser.assets/image-20200423111424865.png)

​		转移矩阵： 

​							![image-20200423111528075](calib_odometer_laser.assets/image-20200423111528075.png)		

​	原优化问题等价的矩阵表示为：![image-20200423111811494](calib_odometer_laser.assets/image-20200423111811494.png)，其中![image-20200423111638862](calib_odometer_laser.assets/image-20200423111638862.png)

   **极大似然问题转化为二次型问题 表示为：**

![image-20200423111938967](calib_odometer_laser.assets/image-20200423111938967.png)

![image-20200423112344103](calib_odometer_laser.assets/image-20200423112344103.png)

具体推到表示如下：
![image-20200423112913438](calib_odometer_laser.assets/image-20200423112913438.png)

  ![image-20200423112727739](calib_odometer_laser.assets/image-20200423112727739.png)
![image-20200423112941914](calib_odometer_laser.assets/image-20200423112941914.png)

![image-20200423112805799](calib_odometer_laser.assets/image-20200423112805799.png)

![image-20200423113000494](calib_odometer_laser.assets/image-20200423113000494.png)
			![image-20200423113526010](calib_odometer_laser.assets/image-20200423113526010.png)



3） 带约束的最小二乘问题：

![image-20200423112059978](calib_odometer_laser.assets/image-20200423112059978.png)

​							![image-20200423112118511](calib_odometer_laser.assets/image-20200423112118511.png)


![image-20200423145327756](calib_odometer_laser.assets/image-20200423145327756.png)

​					对应存在$\lambda $ 使得$M+ \lambda W $为奇异矩阵， 即
​										![image-20200423145459881](calib_odometer_laser.assets/image-20200423145459881.png)

​					其中

![image-20200423145854222](calib_odometer_laser.assets/image-20200423145854222.png)

​					（a）可得到关于$\lambda$的二阶项，项系数为： ![image-20200423145959934](calib_odometer_laser.assets/image-20200423145959934.png)
​						求得系数 $\lambda^{(1)} \lambda^{(2)}  $

​					![image-20200423150024178](calib_odometer_laser.assets/image-20200423150024178.png)

​			(b) 求解参数$\varphi ^{(i)}$

​			$\gamma ^{(i)}$ 是$M + λ^{(i) }W$ 核的非零向量， 也就是$M + λ^{(i) }W = 0 $的通解。  同时依据公式24的约束归一化求得$\varphi ^{(i)}$ ：
​									![image-20200423151228143](calib_odometer_laser.assets/image-20200423151228143.png)

​			（c）通过具体的约束关系 公式25，从$\varphi ^{(1)}$ $\varphi ^{(2)}$ 找到正确的 $\hat \varphi $ 

​         4） 系统参数的解
​									![image-20200423151630561](calib_odometer_laser.assets/image-20200423151630561.png)

## 4 异常剔除

 轮子打滑， 传感器平移估计出错, 以及数据同步异常等原因，需要剔除异常值

利用估计的模型参数计算里程计增量在传感器坐标系下的表示，比较其与传感器计算的观测增量，偏差比较大则为异常值，要剔除。

![image-20200423152408380](calib_odometer_laser.assets/image-20200423152408380.png) 

![image-20200423165202651](calib_odometer_laser.assets/image-20200423165202651.png)



## 5. 不确定性分析： 协方差估计

![image-20200423183919062](calib_odometer_laser.assets/image-20200423183919062.png)



## 6. 离散化 常速度简化形式

![image-20200423172743295](calib_odometer_laser.assets/image-20200423172743295.png)

![image-20200423172809257](calib_odometer_laser.assets/image-20200423172809257.png)

![image-20200423172756519](calib_odometer_laser.assets/image-20200423172756519.png)

![image-20200423172826361](calib_odometer_laser.assets/image-20200423172826361.png)

Bounding the Approximation Error 区间常速度的限制：

![image-20200423173110065](calib_odometer_laser.assets/image-20200423173110065.png)

![image-20200423173119246](calib_odometer_laser.assets/image-20200423173119246.png)

## 7. 实验参考

### 7.1 数据量越多越好

### 7.2 小环境测量更佳

```
With minimal tuning of the maximum velocities and the interval length, one can make the robot stay in a small region.
```

### 7.3 分段输入

### 7.4  相对低速运动： 低速且不要很低

```
Choose commands that lead to relatively low speeds. This minimizes the possibility of slipping and ensures that the sensor data are not perturbed by the robot motion. However, do not choose speeds so low that the nonlinear effects of the dynamics become relevant, especially if using the constant speed assumption (usually robots with DC motors are commanded in velocities via voltage, but the platform does not attain constant velocity instantaneously).

```

### 7.5 时间区间T的选择

T太短，标定会过于敏感，T越长关于参数的信息会越丰富；但T不能过长，不然激光计算的数据的相关性会降低（匹配精度会损失）

作者实验条件参考：Ｔ＝0.8s　　ｗ_max=　0.5 rad/s
==》  T内平移 1 cm   旋转 20°  

laser：５ＨＺ　取4帧

 ```
In our setting, we first chose the maximum wheel speed to be 0.5 rad/s (30 ◦ /s),
which made sure that the robot does not slip on the particular terrain. We recorded range-finder readings at 5 Hz as well as dense odometry readings (at 100 Hz). Then, we used only one in four range readings, which corresponds to choosing an interval of T  0.8 s, such that the robot travels approximately 1 cm (in translation) and 20 ◦ (in rotation) per interval.
 ```

四组运动实验：直线运动， 纯旋转， 朝左右运动

![image-20200423182015673](calib_odometer_laser.assets/image-20200423182015673.png)

github代码daima建议：

建议你采集一小段距离（ 运动 1 分钟 左右）就可以了，距离越长，outlier 越多，需要调整参数，才能有好的标定结果。标定路径不需要闭环，简单跑一下弯曲的轨迹就行，迭代次数可以修改，论文中采用过 4-8之间，效果差不多。



补充： 拉格朗日乘子方法求解带约束的优化问题 [如何理解拉格朗日乘子法？](https://www.zhihu.com/question/38586401)
		![image-20200423144649910](calib_odometer_laser.assets/image-20200423144649910.png)



## 8. 算法伪代码

![image-20200423135713352](calib_odometer_laser.assets/image-20200423135713352.png)



## 9. 参考

论文： As paper [Simultaneous Calibration of Odometry and Sensor Parameters for Mobile Robots](https://www.researchgate.net/publication/260634803_Simultaneous_Calibration_of_Odometry_and_Sensor_Parameters_for_Mobile_Robots)

论文对应代码:  https://github.com/AndreaCensi/calibration

中文参考博客：[2d Laser 和 Odomter 内外参数标定工具原理及使用方法](https://blog.csdn.net/heyijia0327/article/details/88571176)

中文参考博客对应代码：https://github.com/MegviiRobot/OdomLaserCalibraTool



## 10 ys_astrid 实验数据



|                     | 1          | 2          | 3           | 4          | aver   | ref-measure |
| ------------------- | ---------- | ---------- | ----------- | ---------- | ------ | ----------- |
| Axle between wheels | 0.602922   | 0.538422   | 0.549336    | 0.546807   | 0.5449 | 0.53575     |
| LiDAR-odom x        | 0.119317   | 0.123547   | 0.139148    | 0.138092   | 0.1336 | 0.14        |
| LiDAR-odom y        | 0.0435663  | 0.00500196 | 0.00582043  | 0.0092771  | 0.0067 | 0.0         |
| LiDAR-odom yaw      | 0.00382949 | 0.00594876 | 0.000741958 | 0.00450296 | 0.0037 | 0.0         |
| Left wheel radius   | 0.083863   | 0.0834078  | 0.0841896   | 0.0837314  | 0.0838 | 0.0845      |
| Right wheel radius  | 0.0871739  | 0.0834341  | 0.0838561   | 0.0834509  | 0.0836 | 0.0845      |
| 数据是否可靠        | N          | y          | y           | y          |        |             |