include "my_backpack_2d.lua"
 
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3, //最大保存子图数，存定位模式通过子图进行定位，但只需要当前和上一个子图即可，我这里设置的是2
}
POSE_GRAPH.optimize_every_n_nodes = 20 //每20个有效帧一个子图，子图构建完成要闭环检测一次，这个数越小，闭环检测越频繁，当然CPU爆炸