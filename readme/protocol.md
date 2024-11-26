# <span style="color:#FCDB72;">QT上位机与ROS端通信协议说明</span>
上位机作为TCP客户端，输入PC的IP地址和端口号(8899)连接机械臂，机械臂作为TCP服务器，端口号为8899.

| data[0] | data[1] | data[2] | data[3] |    Operation   |
|  :---:  |  :---:  |  :---:  |  :---:  |      :---:     |
|  0x00   |  0x00   |   0x00  |   0x01  |      Record    |
|  0x00   |  0x00   |   0x00  |   0x02  |   Stop Record  |
|  0x00   |  0x00   |   0x00  |   0x03  |    Replaying   |
|  0x00   |  0x00   |   0x00  |   0x04  |      Enable    |
|  0x00   |  0x00   |   0x00  |   0x05  |     Disable    |
|  0x00   |  0x00   |   0x00  |   0x06  | Emergency Stop |
|  0x00   |  0x00   |   0x00  |   0x07  |  Initial Pose  |

| data[0] | data[1] | data[2] | data[3] |    Operation   |
|  :---:  |  :---:  |  :---:  |  :---:  |      :---:     |
|  0x01   |  0x01   |   0x00  |  value  |  z pos - value |
|  0x01   |  0x01   |   0x01  |  value  |  z pos + value |
|  0x01   |  0x02   |   0x00  |  value  |  z rot - value |
|  0x01   |  0x02   |   0x01  |  value  |  z rot + value |
|  0x02   |  0x01   |   0x00  |  value  |  x pos - value |
|  0x02   |  0x01   |   0x01  |  value  |  x pos + value |
|  0x02   |  0x01   |   0x00  |  value  |  x rot - value |
|  0x02   |  0x01   |   0x01  |  value  |  x rot + value |
|  0x03   |  0x01   |   0x00  |  value  |  y pos - value |
|  0x03   |  0x01   |   0x01  |  value  |  y pos + value |
|  0x03   |  0x02   |   0x00  |  value  |  y rot - value |
|  0x03   |  0x02   |   0x01  |  value  |  y rot + value |

| data[0] | data[1] | data[2] | data[3] | data[4] | data[5] | data[6] | data[7] | data[8] | data[9] |
|  :---:  |  :---:  |  :---:  |  :---:  |  :---:  |  :---:  |  :---:  |  :---:  |  :---:  |  :---:  |
| 0x0479  |  joint1 |  joint2 |  joint3 |  joint4 |  joint5 |  joint6 | gripper_pos | gripper_vel | gripper_force |

 
# <span style="color:#FCDB72;">FDCAN转RS485夹爪通信协议说明</span>
| data[0] | data[1] | data[2] | data[3] | data[4] | data[5] | data[6] | 
|  :---:  |  :---:  |  :---:  |  :---:  |  :---:  |  :---:  |  :---:  |
|   0x55  |   0xAA  |  enable |   pos   |   vel   |  force  |   0xEB  |