#使用方法
roslaunch local_planner local_planner.launch 
#适配自己的机器人
修改launch/local_planner.launch中的定位话题和点云话题
points_topic 为自己的点云，（base_link)坐标系，最好是滤除地面点后的点云；
state_topic 修改为自己的定位话题
