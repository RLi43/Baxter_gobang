# Baxter_gobang
Baxter robot play the gobang with human. #CV#gobang#Baxter#ROS#Matlab

## Nodes & Topics

### Nodes

* gobang_ai
* arm_controller
  * measure the position of chess board
  * grip chesses by the hand camera
  * place chess
* image_processor
* face - show picture on xdisplay
* head - 

### Topics

* Info/board ChessBoard
  * Information about chess board / 棋盘信息，初始位置信息
  * Pub : [image_processor]
  * Sub : arm_controller
* posi/human ChessMove
  * human move, incudes withdraw/change difficulty / 人下的位置，可选参数包括重新开始，改变难度和悔棋（相应操作未实现）
  * Pub : [image_processor]
  * Sub : gobang_ai
* posi/ai Int16MultiArray(Better change it to custom type)
  * ai_move / ai计算得的下一步位置
  * Pub : gobang_ai
  * Sub : arm_controller
* ai_state - waiting/thinking/win/lose - for face photos
  * Pub : gobang_ai, arm_controller, image_processor
  * Sub : face



## Referrence

Andrew Tzer-Yeu Chen and Kevin I-Kai Wang ,Computer Vision Based Chess Playing Capabilities for the Baxter Humanoid Robot 

###### gobang AI

mostly from https://github.com/skywind3000/gobang, modified it to fit ROS

###### arm_controller

* movement - baxter demo ‘pick and place’
* graph process - [Baxter抓取物块——基于单应性矩阵（一）](<https://blog.csdn.net/Hey_chaoxia/article/details/81914729>)

###### face

baxter_tool
## Pictures
https://cloud.tsinghua.edu.cn/d/38ad384fa774416fab5a/

## Next

* when to deal with the board to get information?
* ik needs to be modify. not just thrust into the board…

## Problems

* Internet delay causes gripping process unstable – it will keep move as the camera photo is not updated