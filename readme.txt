1、安装vrep
正常的话解压就可以用
./coppeliasim.sh 正常可以直接打开软件
然后选择工程中的，我调好的小车模型

2、编译运行
有三个工程文件
main_ws :是主体工程
matplotxxx：是C++调的python,主要是绘图用的，在demo里没调
vrepxxx：是针对vrep写的小车程序，完成了对vrep机器人的信息交互，gps 和 imu等，并且在main_ws我已经写好了读和写，你直接编译，小车应该就能动

3、小车能动
后续具体的速度关系 自己映射下就行  一个方便好用的小移动机器人
