# SDK

## SDK简介

协作机器人软件开发工具包（Software Development Kit，SDK）是一套面向开发者和系统集成商的工具和资源，旨在帮助用户快速、高效地构建与协作机器人相关的应用程序。通过SDK，用户可以轻松实现机器人控制、任务编排、传感器集成以及与其他设备或系统的交互。

JAKA SDK 定义了一套全面的API以实现对JAKA机器人的交互和控制，它为客户开发自己的应用程序来连接JAKA机器人控制器提供了一种方式。

#### 文档须知

在编写本文档时，我们预期读者应具有以下基础：

- 熟悉JAKA机器人，对机器人控制有基本的了解；
- 能够在目标操作系统Linux上使用C/C++编程语言。

#### 适配版本

此文档适配的SDK版本为 V2.3.0_DUAL。

SDK适配 Linux x86_64 和  Amd 64 系统。

适配的机器人系统内版本信息如下表：

|        名称        |       版本号        | 其他信息 |
| :----------------: | :-----------------: | :------: |
|       控制器       |     3.3.0_beta      |    /     |
|        SCB         |       1.3.5.0       |    /     |
| EtherCAT伺服上位机 |       1.0.11        |    /     |
|        伺服        | ECAT_ServoDrive_318 |    /     |
|        APP         |   K1_alpha5_3.3.1   |    /     |



#### 注意事项

- 节卡SDK使用的的长度单位统一为毫米（mm），角度单位统一为弧度（rad）；
- JAKA使用的SDK编码方式为UTF-8编码；
- 动态库的安装和开发环境搭建，请参考[动态库使用](https://www.jaka.com/docs/guide/SDK/Dev_env_construt.html)；
- 在调用接口前必须先登录，调用login_in()；
- 在调用接口下发指令前，请注意检查上条指令接口的返回值，是否正确返回。



## C++

本文档将介绍JAKA SDK（C++）中定义的数据类型和API，主要适用于使用C/C++创建与虚拟或真实控制器通信的机器人应用程序的软件开发人员。对于需要了解JAKA机器人控制器应用程序的用户也会有一定帮助。

### 机械臂基础

以下示例默认包含了必要的头文件，包含"JAKAZuRobot.h", C++原生的头文件。

#### 机械臂控制类构造函数

连接机械臂控制器，该接口启动成功，是其他接口调用的基础

- 参数说明
  - ip 控制器ip地址
- 返回值 ERR_SUCC 成功 其他失败

```C++
JAKAZuRobot(); 
```



#### 机械臂登录

连接机械臂控制器，该接口启动成功，是其他接口调用的基础。

- 参数说明
  - ip 控制器ip地址
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t login_in(const char *ip, bool grpc_flag = false);
```



#### 机械臂注销

断开控制器连接

- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t login_out();
```



#### 机械臂上电

打开机械臂电源

- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t power_on();
```



#### 机械臂下电

关闭机器人电源

- 返回值 ERR_SUCC 成功 其他失败

```C++
errno_t power_off();
```



#### 机械臂关机

机器人控制柜关机

- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t shut_down();
```
注意：
1. 若本体使能会先关闭使能
2. 若本体上电会先关闭电源
3. 该接口耗时约3～5s，调用在3s时刻左右会确认保存配置，期间禁止其他接口调用，且保持控制柜供电，否则会有概率导致文件系统损坏，用户配置丢失
4. 调用完成后SDK连接会断开



#### 机械臂上使能

控制机器人上使能

- 返回值 ERR_SUCC 成功 其他失败

```C++
errno_t enable_robot();
```



#### 机械臂下使能

控制机器人下使能

- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t disable_robot();
```



#### 控制机械臂进入或退出拖拽模式

控制机器人进入或退出拖拽模式

- 参数说明
  - enable  TRUE为进入拖拽模式，FALSE为退出拖拽模式
- 返回值 ERR_SUCC 成功 其他失败

```C++
errno_t drag_mode_enable(int robot_id, BOOL enable);
```

代码示例：

```C++
#include <iostream> 
#include "JAKAZuRobot.h" 


//拖拽模式 
int example_drag() 
{ 
    BOOL in_drag[2]; 
    JAKAZuRobot demo; 
    demo.login_in("192.168.2.152"); 
    demo.power_on(); 
    demo.enable_robot(); 
    //确认机械臂左臂是否在拖拽模式下 
    demo.is_in_drag_mode(in_drag); 
    std::cout << "left robot is in drag: " <<  is_drag[0] << " right robot is in drag: " << is_drag[1] << std::endl; 
    //使能拖拽模式 
    demo.drag_mode_enable(0, TRUE); 
    Sleep(10000);
    demo.is_in_drag_mode(in_drag); 
    std::cout << "left robot is in drag: " <<  is_drag[0] << " right robot is in drag: " << is_drag[1] << std::endl;
    //去使能拖拽模式 
    demo.drag_mode_enable(0, FALSE); 
    Sleep(100); 
    demo.is_in_drag_mode(&in_drag); 
    std::cout << "left robot is in drag: " <<  is_drag[0] << " right robot is in drag: " << is_drag[1] << std::endl; 
    while (1) 
    { 
        demo.is_in_drag_mode(&in_drag); 
        std::cout << "left robot is in drag: " <<  is_drag[0] << " right robot is in drag: " << is_drag[1] << std::endl; 
        Sleep(100); 
    } 
    return 0;
} 
```



#### 查询机械臂是否处于拖拽模式

查询机器人是否处于拖拽模式

- 参数说明
  - in_drag 查询结果
- 返回值 ERR_SUCC 成功 其他失败

```C++
errno_t is_in_drag_mode(BOOL *in_drag);
```



#### 获取SDK版本号

获取SDK版本号

- 参数说明
  - version SDK版本号
- 返回值 ERR_SUCC 成功 其他失败

```C++
errno_t get_sdk_version(char *version);
```

代码示例：

```c++
#include <iostream> 
#include "JAKAZuRobot.h" 

 
int example_getsdk_version() 
{ 
    //实例API对象demo  
    JAKAZuRobot demo; 
    char ver[100]; 
    //登陆控制器，需要将192.168.2.194替换为自己控制器的IP 
    demo.login_in("192.168.2.194"); 
    //查询当前SDK版本 
    demo.get_sdk_version(ver); 
    std::cout << " SDK version is :" << ver << std::endl; 
    return 0; 
} 
```



#### 设置SDK日志路径

设置SDK日志路径

- 参数说明
  - filepath SDK日志路径
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t set_SDK_filepath(const char *filepath);
```

代码示例

```c++
#include <iostream> 
#include "JAKAZuRobot.h" 


//设置SDK日志路径 
int example_set_SDK_filepath() 
{ 
    //设置SDK日志路径 
    char path[20] = "D://test.log"; 
    int ret; 
    JAKAZuRobot demo; 
    ret = demo.set_SDK_filepath(path);//设置SDK文件路径 
    demo.login_in("192.168.2.194"); 
    demo.power_on(); 
    demo.enable_robot(); 
    std::cout << ret << std::endl; 
    return 0; 
} 
```



#### 设置SDK日志路径（静态方法）

- 参数说明
  - filepath SDK日志路径
- 返回值 ERR_SUCC 成功 其他失败

```C++
static errno_t static_Set_SDK_filepath(const char *filepath);

```

#### 设置SDK是否开启调试模式

设置是否开启调试模式，选择TRUE时，开始调试模式，此时会在标准输出流中输出调试信息，选择FALSE时，不输出调试信息

- 参数说明
  - mode TRUE开启 FALSE关闭
- 返回值 ERR_SUCC 成功 其他失败

```C++
errno_t set_debug_mode(BOOL mode);
```

代码示例：

```c++
#include <iostream> 
#include "JAKAZuRobot.h" 

 
//设置SDK是否开启调试模式 
int example_set_debug_mode() 
{ 
    BOOL mode;  
    JAKAZuRobot demo; 
    //设置调试模式，将在终端上打印调试信息 
    demo.set_debug_mode(TRUE); 
    demo.login_in("192.168.2.194"); 
    demo.power_on(); demo.enable_robot();
    return 0; 
} 
```



### 机械臂运动

#### 多机器人同步运动指令movj

多机器人同步运动指令

- 参数说明
  - robot_id 机器人ID 接受LEFT(0) RIGHT(1) DUAL(-1) 
  - move_mode ABS(绝对运动) or INCR（相对运动）
  - is_block TRUE(阻塞运动) or FALSE(非阻塞运动)
  - joint_pos 两个机器人的位置指令
  - vel 两个机器人速度指令
  - acc 两个机器人的加速度指令
  - tol 两个机器人的轨迹转接时允许误差，范围>=0
- 返回值 ERR_SUCC 成功 其他失败

```C++
errno_t robot_run_multi_movj(int robot_id, const MoveMode *move_mode, BOOL is_block, const JointValue *joint_pos, const double* vel, const double* acc, const double* tol = nullptr);
```



#### 多机器人同步运动指令movl

多机器人同步运动指令

- 参数说明
  - robot_id 机器人ID 接受LEFT(0) RIGHT(1) DUAL(-1) 
  - move_mode ABS(绝对运动) or INCR（相对运动）
  - is_block TRUE(阻塞运动) or FALSE(非阻塞运动)
  - end_pos 两个机器人的位置指令
  - vel 两个机器人速度指令
  - acc 两个机器人的加速度指令
  - tol 两个机器人的轨迹转接时允许误差，范围>=0
- 返回值 ERR_SUCC 成功 其他失败

```C++
errno_t robot_run_multi_movl(int robot_id, const MoveMode *move_mode, BOOL is_block, const JointValue *end_pos, const double* vel, const double* acc, const double* tol = nullptr);
```



#### 机械臂运动终止

终止当前机械臂运动

- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t motion_abort();
```



#### 获取两个机器是否到位

获取两个机器人是否到位

- 参数说明
  - inpos 1 到位； 0 未到位； 2维
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_is_inpos(int* inpos);
```

代码示例：

```C++
#include <iostream> 
#include "JAKAZuRobot.h" 
#define PI 3.1415926 

//查询机械臂运动是否停止 
int example_is_in_pos() 
{ 
    //实例API对象demo  
    JAKAZuRobot demo; 
    BOOL in_pos; 
    //登陆控制器，需要将192.168.2.194替换为自己控制器的IP 
    demo.login_in("192.168.2.152"); 
    //机械臂上电 
    demo.power_on(); 
    //机械臂上使能 
    demo.enable_robot(); 
    while (1) 
    { 
        //查询是否运动停止 
        demo.is_in_pos(&in_pos); 
        if (inpos != last_inpos)
            std::cout << "jval is " << cur_jval << "{" << (inpos ? "True" : "False") << ","<< (last_inpos ? "True" : "False") << "}" << '\n';
        last_inpos = inpos;
    } 
    return 0; 
} 
```



#### 机械臂设置阻塞运动超时时间

设置机器人阻塞等待超时时间

- 参数说明
  - seconds 时间参数，单位秒
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t set_block_wait_timeout(float seconds);
```



### 机械臂操作信息设置与获取

#### 获取机械臂状态

获取机械臂状态

- 参数说明
  - state 机械臂状态查询结果，双臂状态与结果；即均为上电才认为上电，均使能才认为使能
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t get_robot_state(RobotState* state); 
```

代码示例：

```C++
#include <iostream> 
#include "JAKAZuRobot.h" 


//获取机械臂状态(急停 上电 伺服使能) 
int example_get_robstate() 
{ 
    JAKAZuRobot demo; 
    //声明机械臂状态结构体 
    RobotState state; 
  	demo.login_in("192.168.2.152"); 
 	demo.power_on(); 
    demo.enable_robot();
    //查询机械臂状态 
    demo.get_robot_state(&state); 
    std::cout << "is e_stoped : " << state.estoped << std::endl; 
    std::cout << "is powered : " << state.poweredOn << std::endl; 
  	std::cout << "is servoEnabled : " << state.servoEnabled << std::endl; 
    return 0; 
} 

```

#### 设置机器人工具坐标偏移

设置机器人工具坐标偏移

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1）
  - tool 机器人工具坐标偏移
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_set_tool_offset(int robot_id, CartesianPos* offset);
```



#### 获取机器人工具坐标偏移

获取机器人工具坐标偏移

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1）
  - tool 机器人工具坐标偏移
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t  robot_get_tool_offset(int robot_id, CartesianPos* offset);
```



#### 获取机器人默认基座标系

获取机器人默认基座标系

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1）
  - bsse_offset 机器人基座标相较于世界坐标的变换，姿态按照ZYX欧拉角描述，单位mm和rad 
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_get_default_base(int robot_id, CartesianPos* base_offset);
```



#### 设置机器人安装位置

获取机器人默认基座标系

- 参数说明
  - angleX 绕x轴的旋转角度,单位rad 
  - angleZ 绕Y轴的旋转角度,单位rad 
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1）
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t set_installation_angle(double angleX, double angleZ, int robot_id = 0);
```



#### 获取机器人安装位置

获取机器人默认基座标系

- 参数说明
  - quat 四元数,表示旋转姿态
  - appang  RPY 欧拉角,表示旋转姿态
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1）
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t get_installation_angle(Quaternion* quat, Rpy* appang, int robot_id = 0);
```



### 机械臂状态设置与查询

#### 查询机械臂是否超出限位

查询机械臂是否超出限位

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1）
  - limit 0 代表无超出，1代表超出
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_is_on_soft_limit(int robot_id, int* limit);
```

代码示例：

```C++
#include <iostream> 
#include "JAKAZuRobot.h" 


//查询是否超出限位 
int example_is_on_limit() 
{ 
    JAKAZuRobot demo; 
    BOOL on_limit; 
    demo.login_in("192.168.2.152"); 
    demo.power_on(); 
    demo.enable_robot(); 
    while (1) 
    { 
        //查询是否超限 
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        int limit = -1;
        ret = robot.robot_is_on_soft_limit(LEFT, &limit);
        printf("left limit: %d\n", limit);
        ret = robot.robot_is_on_soft_limit(RIGHT, &limit);
        printf("right limit: %d\n", limit);
        ret = robot.robot_is_on_soft_limit(DUAL, &limit);
        printf("dual limit: %d\n", limit);
    } 
    return 0; 
} 
```



#### 查询机械臂是否处于碰撞保护模式

查询机械臂是否处于碰撞保护模式

- 参数说明
  - in_collision 查询结果 
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t is_in_collision(Bool* in_collision); 
```



#### 碰撞之后从碰撞保护模式恢复

碰撞之后从碰撞保护模式恢复 

- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t collision_recover();
```

代码示例：

```c++
#include <iostream> 
#include "JAKAZuRobot.h" 


//碰撞保护状态查询，恢复 
int example_collision_recover() 
{ 
    JAKAZuRobot demo; 
    BOOL in_collision; 
    demo.login_in("192.168.2.152"); 
	demo.power_on(); 
	demo.enable_robot(); 
	//查询是否处于碰撞保护状态 
	demo.is_in_collision(&in_collision); 
	std::cout << " in_collision is :" << in_collision << std::endl; 
	if (in_collision) 
        //如果处于碰撞保护模式，则从碰撞保护中恢复
    {
        demo.collision_recover();
    } 
    else{
        std::cout << "robot is not collision" << std::endl;
    } 
    return 0; 
} 
```



#### 设置机械臂碰撞等级

设置机械臂碰撞等级 

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1）
  - level 碰撞等级，取值范围[0,5] ，其中0为关闭碰撞，1为碰撞阈值25N，2为碰撞阈值50N，3为碰撞阈值75N，4为碰撞阈值100N，5为碰撞阈值125N
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t set_collision_level(int robot_id， const int level);
```

代码示例：

```C++
#include <iostream> 
#include "JAKAZuRobot.h" 


//碰撞等级查看，设置 
int example_collision_level() 
{ 
	//实例API对象demo  
	JAKAZuRobot demo; 
	int level;
	//登陆控制器，需要将192.168.2.194替换为自己控制器的IP 
	demo.login_in("192.168.2.152"); 
	//机械臂上电 
	demo.power_on(); 
	//机械臂上使能 
	demo.enable_robot(); 
	//查询当前碰撞等级 
	demo.get_collision_level(LEFT, &level); 
	std::cout << " collision level is :" << level << std::endl; 
	//设置碰撞等级，[0,5]，0为关闭碰撞，1为碰撞阈值25N，2为碰撞阈值50N，3为碰撞阈值75N，4为碰撞阈值100N，5为碰撞阈值125N， 
	demo.set_collision_level(LEFT, 2); 
	//查询当前碰撞等级 
	demo.get_collision_level(LEFT, &level); 
	std::cout << " collision level is :" << level << std::endl; 
	return 0; 
} 
```



#### 获取机械臂碰撞等级

获取机械臂设置的碰撞等级

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1）
  -  level 碰撞等级
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t get_collision_level(int robot_id, int* level);
```



#### 设置网络异常时机械臂自动终止运动类型

设置网络异常控制句柄，SDK与机械臂控制器失去连接后多长时间机械臂控制器终止机械臂当前运动

- 参数说明
  - millisecond 时间参数，单位：ms
  - mnt 网络异常时机械臂需要进行的动作类型
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t set_network_exception_handle(float millisecond, ProcessType mnt);
```

代码示例：

```C++
#include <iostream> 
#include "JAKAZuRobot.h" 


//设置网络异常时机械臂自动终止运动类型 
int example_set_network_exception_handle() 
{ 
    float milisec = 100; 
    int ret; 
    JAKAZuRobot demo; 
    demo.login_in("192.168.2.194"); 
    demo.power_on(); 
    demo.enable_robot(); 
    //设置柔顺力矩条件 
    ret = demo.set_network_exception_handle(milisec, MOT_KEEP); 
    std::cout << ret << std::endl; 
    return 0; 
} 
```



#### 机器人错误状态清除

错误状态清除

- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t clear_error();
```



#### 获取机械臂目前发生的最后一个错误码

获取机械臂运行过程中最后一个错误码,当调用clear_error时，最后一个错误码会清零

- 参数说明
  - code 机械臂的错误码
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t get_last_error(ErrorCode* code);
```



#### 设置机械臂错误码文件存放路径

设置错误码文件路径，需要使用get_last_error接口时需要设置错误码文件路径，如果不使用get_last_error接口，则不需要设置该接口

- 参数说明
  - path 机械臂错误码文件存放路径
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t set_errorcode_file_path(char* path);
```

代码示例：

```C++
#include <iostream> 
#include "JAKAZuRobot.h" 


//错误码查看 
int example_get_last_errcode() 
{ 
    int ret; 
    //初始化错误码文件存放路径 
    char path[100] = "E:\JAKA_ERROR_CODE.csv";  
    JAKAZuRobot demo; 
    ErrorCode Eret; 
    demo.login_in("192.168.2.194"); 
    demo.power_on(); 
    demo.enable_robot(); 
    ret = demo.program_load("not_exist999875");//故意加载一个不存在的程序，引发报错。 
    std::cout << ret << std::endl; 
    demo.get_last_error(&Eret);//查询最后一个报错信息 
    std::cout << " error code is :" << Eret.code << " message: "<< Eret.message<< std::endl; 
    demo.set_errorcode_file_path(path);//设置错误码说明文件 
    demo.get_last_error(&Eret);//查询最后一个报错信息 
    std::cout << " error code is :" << Eret.code << " message: " << Eret.message << std::endl; 
    return 0; 
} 
```



#### 确认机器人是否处于错误状态

确认机器人是否处于错误状态 

- 参数说明
  -  in_error 0代表机器人处于正常状态，1代表机器人处于错误状态
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_is_in_error(int* in_error);
```



### 机械臂伺服运动模式

#### 机械臂伺服初始化功能

启用 EDG功能。只有在启用该功能后，才能调用与 EDG 相关的所有接口。
- 参数说明
  - en 开关标志。true 表示启用 EDG 功能，false 表示关闭
  - edg_stat_ip  SDK 客户端的 IP 地址（用于接收 EDG 反馈数据）
  - edg_port  SDK 客户端用于接收 EDG 反馈数据的端口号
  - edg_mode  EDG 模式选择：
    - 0：所有 EDG 相关接口均可调用
    - 1：除 edg_servo_j 与 edg_servo_p 外，其他接口均可调用
  - edg_prio  EDG 线程优先级，仅 Linux 有效
  - edg_cpuid    EDG 线程绑定的 CPU 核 ID，仅 Linux 有效
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t edg_init(BOOL en = true, const char* edg_stat_ip = "0.0.0.0", int edg_port = 10010, int edg_mode = 0);
```



#### 机械臂伺服位置控制模式使能

机械人伺服SERVO MOVE模式使能

- 参数说明
  - enable TRUE为进入SERVO MOVE模式，FALSE表示退出该模式 
  - is_block TRUE(阻塞模式) or FALSE(非阻塞模式）
  - robindex 机器人ID 接收LEFT(0) RIGHT(1）
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t servo_move_enable(BOOL enable，BOOL is_block = true，int robindex = 0);
```



#### 机械臂关节空间伺服模式运动

实时以固定周期发送关节位置指令，单位rad

- 参数说明
  - robot_index 机器人索引号，-1表示所有机器人
  - joint_pos 机械臂关节运动目标位置
  - move_mode 指定运动模式：增量运动、绝对运动和连续运动
  - step_num 步数，表示发送指令的步数，默认1步。比如调用周期无法保证1ms，则可以设置为更大值，比如2，则调用周期可允许为2ms。主要用于客户端实时性不好的场景，容许更大的抖动。或处理周期需要更长的情况
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t edg_servo_j(unsigned char robot_index, const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num=1);
```



#### 机械臂笛卡尔空间伺服模式运动

实时以固定周期发送笛卡尔位置指令，单位rad和mm

- 参数说明
  - robot_index 机器人索引号，-1表示所有机器人
  - cartesian_pose 机械臂关节运动目标位置
  - move_mode 指定运动模式：增量运动、绝对运动和连续运动
  - step_num 步数，表示发送指令的步数，默认1步。比如调用周期无法保证8ms，则可以设置为更大值，比如2，则调用周期可允许为16ms。主要用于客户端实时性不好的场景，容许更大的抖动。或处理周期需要更长的情况
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t edg_servo_p(unsigned char robot_index, const CartesianPose* cartesian_pose,  MoveMode move_mode, unsigned int step_num=1);
```



#### 机械臂SERVO模式下禁用滤波器

SERVO模式下不使用滤波器,该指令在SERVO模式下不可设置，退出SERVO后可设置

- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t servo_move_use_none_filter();
```

代码示例：

```C++
#include <iostream> 
#include "JAKAZuRobot.h" 

//servo模式禁用滤波器 
int example_servo_use_none_filter() 
{ 
    int ret; 
    //实例API对象demo  
    JAKAZuRobot demo; 
    //登陆控制器，需要将192.168.2.194替换为自己控制器的IP 
    demo.login_in("192.168.2.194"); 
    //机械臂上电 
    demo.power_on(); 
    //机械臂上使能 
    demo.enable_robot(); 
    ret = demo.servo_move_use_none_filter(); 
    std::cout << ret << std::endl; 
    return 0; 
} 
```



#### 机械臂SERVO模式下关节空间一阶低通滤波

SERVO模式下切换到关节空间一阶低通滤波,该指令在SERVO模式下不可设置，退出SERVO后可设置

- 参数说明
  - cutoffFreq 一阶低通滤波器截止频率
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t servo_move_use_joint_LPF(double cutoffFreq);
```

代码示例：

```c++
#include <iostream> 
#include "JAKAZuRobot.h" 


//servo模式下关节空间一阶低通滤波 
int example_servo_use_joint_LPF() 
{ 
    int ret; 
    //实例API对象demo  
    JAKAZuRobot demo; 
    //登陆控制器，需要将192.168.2.194替换为自己控制器的IP 
    demo.login_in("192.168.2.194");
    //机械臂上电 
    demo.power_on(); 
    //机械臂上使能 
    demo.enable_robot(); 
    //servo模式下关节空间一阶低通滤波,截止频率0.5Hz 
    ret = demo.servo_move_use_joint_LPF(0.5); 
    std::cout << ret << std::endl; 
    return 0; 
} 
```



#### 机械臂SERVO模式下关节空间非线性滤波

SERVO模式下切换到关节空间非线性滤波,该指令在SERVO模式下不可设置，退出SERVO后可设置

- 参数说明
  - max_vr 笛卡尔空间姿态变化速度的速度上限值（绝对值）°/s
  - max_ar 笛卡尔空间姿态变化速度的加速度上限值（绝对值）°/s^2 
  - max_jr 笛卡尔空间姿态变化速度的加加速度上限值（绝对值）°/s^3
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t servo_move_use_joint_NLF(double max_vr, double max_ar, double max_jr);
```

代码示例：

```C++
#include <iostream> 
#include "JAKAZuRobot.h" 


//servo模式下关节空间非线性滤波 
int example_servo_use_joint_NLF() 
{ 
    int ret; 
    //实例API对象demo  
    JAKAZuRobot demo; 
    //登陆控制器，需要将192.168.2.194替换为自己控制器的IP 
    demo.login_in("192.168.2.194"); 
    //机械臂上电 
    demo.power_on(); 
    //机械臂上使能 
    demo.enable_robot(); 
    //servo模式下关节空间非线性滤波 
    ret = demo.servo_move_use_joint_NLF(2,2,4); 
    std::cout << ret << std::endl; 
    return 0; 
} 
```



#### 机械臂SERVO模式下笛卡尔空间非线性滤波

SERVO模式下切换到笛卡尔空间非线性滤波,该指令在SERVO模式下不可设置，退出SERVO后可设置

- 参数说明
  - max_vp 笛卡尔空间下移动指令速度的上限值（绝对值）。单位：mm/s]()
  - max_ap 笛卡尔空间下移动指令加速度的上限值（绝对值）。单位：mm/s^2
  - max_jp 笛卡尔空间下移动指令加加速度的上限值（绝对值）单位：mm/s^3
  - max_vr 笛卡尔空间姿态变化速度的速度上限值（绝对值）°/s
  - max_ar 笛卡尔空间姿态变化速度的加速度上限值（绝对值）°/s^2
  - max_jr 笛卡尔空间姿态变化速度的加加速度上限值（绝对值）°/s^3
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t servo_move_use_carte_NLF(double max_vp, double max_ap, double max_jp, double max_vr, double max_ar, double max_jr);
```

代码示例：

```C++
#include <iostream> 
#include "JAKAZuRobot.h" 


//servo模式下笛卡尔空间非线性滤波 
int example_servo_use_carte_NLF() 
{ 
    int ret; 
    //实例API对象demo  
    JAKAZuRobot demo; 
    //登陆控制器，需要将192.168.2.194替换为自己控制器的IP 
    demo.login_in("192.168.2.194"); 
    //机械臂上电 
    demo.power_on(); 
    //机械臂上使能 
    demo.enable_robot(); 
    //servo模式下笛卡尔空间非线性滤波 
    ret = demo.servo_move_use_carte_NLF(2, 2, 4, 2, 2, 4); 
    std::cout << ret << std::endl; 
    return 0; 
} 
```



#### 机械臂SERVO模式下关节空间多阶均值滤波

SERVO模式下切换到关节空间多阶均值滤波器,该指令在SERVO模式下不可设置，退出SERVO后可设置

- 参数说明
  - max_buf 均值滤波器缓冲区的大小
  - kp 加速度滤波系数
  - kv 速度滤波系数
  -  ka 位置滤波系数
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t servo_move_use_joint_MMF(int max_buf, double kp, double kv, double ka);
```

代码示例：

```C++
#include <iostream> 
#include "JAKAZuRobot.h" 

#define PI 3.1415926 

//servo模式下关节空间多阶均值滤波 
int example_servo_use_joint_MMF() 
{ 
    int ret; 
    //实例API对象demo  
    JAKAZuRobot demo; 
    //登陆控制器，需要将192.168.2.194替换为自己控制器的IP 
   	demo.login_in("192.168.2.194"); 
    //机械臂上电 
    demo.power_on(); 
    //机械臂上使能 
    demo.enable_robot();
    //servo模式下关节空间多阶均值滤波 
    ret = demo.servo_move_use_joint_MMF(20, 0.2, 0.4, 0.2); 
    std::cout << ret << std::endl; 
    return 0; 
} 
```



#### 机械臂SERVO模式下速度前瞻参数设置

SERVO模式下速度前瞻参数设置 

- 参数说明
  - max_buf 均值滤波器缓冲区的大小
  - kp 加速度滤波系数
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t servo_speed_foresight(int max_buf, double kp);
```

代码示例：

```c++
#include <iostream> 
#include "JAKAZuRobot.h" 


//servo模式下速度前瞻参数设置 
int example_speed_foresight() 
{ 
    int ret; 
    //实例API对象demo  
    JAKAZuRobot demo; 
    //登陆控制器，需要将192.168.2.194替换为自己控制器的IP 
    demo.login_in("192.168.2.194"); 
    //机械臂上电 
    demo.power_on(); 
    //机械臂上使能 
    demo.enable_robot(); 
    //servo模式下关节空间多阶均值滤波 
    ret = demo.servo_speed_foresight(200, 2); 
    std::cout << ret << std::endl; 
    return 0; 
} 
```



#### 获取机器人状态

获取机器人状态，此接口获取的为缓存数据

- 参数说明
  - robot_index 机器人索引号，接收LEFT(0) RIGHT(1)
  - joint_pos  关节位置，单位rad
  - cartesian_pos 笛卡尔位置，单位mm和rad
  - sensor_torque 传感器扭矩，单位Nm
  -  joint_torque 关节扭矩，单位Nm
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t edg_get_stat(unsigned char robot_index, JointValue *joint_pos, CartesianPos *cartesian_pos, CartesianPos *sensor_torque= nullptr, CartesianPos *joint_torque=nullptr);
```



#### 获取机器人状态详细信息

获取机器人详细状态信息，此接口获取的为缓存数据 

- 参数说明
  - details 状态详细信息
- 返回值 ERR_SUCC 成功 其他失败
- 该接口即将废弃，数据内容计划与edg_get_stat合并

```c++
errno_t edg_stat_details(unsigned long int details[3]);
```



#### 获取机器人末端FREE按钮状态

获取机器人末端FREE按钮状态

- 参数说明
  - keys 末端FREE按钮状态，0表示未按下，1表示按下。分别代表两个机器人的
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t edg_stat_free_key(bool keys[2]);
```



#### EDG接收

周期任务中触发内部接收事件并刷新缓存

- 参数说明
  - next 下一个状态数据的时间戳
- 返回值 ERR_SUCC 成功 其他失败
- 该接口即将废弃，可以不用在周期任务中调用

```c++
errno_t edg_recv(struct timespec *next=nullptr);
```



#### EDG发送

周期任务中触发内部发送事件，搭配edg_servo_p和edg_servo_j使用，只有调用edg_send后，才会真正发送指令

- 参数说明
  - cmd_index 指令索引，用于标识指令的唯一性，如果为nullptr，则内部默认为自增
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t edg_send(const uint32_t *cmd_index=nullptr);
```



### 机械臂运动学

#### 机械臂求解逆解

计算指定位姿在当前工具、当前安装角度以及当前用户坐标系设置下的逆解

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1)
  - ref_pos 逆解计算用的参考关节空间位置
  - cartesian_pose 笛卡尔空间位姿值
  - joint_pos 计算成功时关节空间位置计算结果
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t kine_inverse(int robot_id, const JointValue* ref_pos, const CartesianPose* cartesian_pose, JointValue* joint_pos);
```

代码示例：

```C++
#include <iostream> 
#include "JAKAZuRobot.h" 


// 机械臂逆解 已知tcp_pos,求joint_pos 

int example_kine_inverse() 
{ 
    int ret; 
    JAKAZuRobot demo; 
    //初始化参考点 
    JointValue ref_jpos = { 0.558, 0.872, 0.872 , 0.349, 0.191, 0.191 }; 
    //初始化笛卡尔空间点坐标 
    CartesianPose tcp_pos; 
    tcp_pos.tran.x = 243.568; tcp_pos.tran.y = 164.064; tcp_pos.tran.z = 742.002;
    tcp_pos.rpy.rx = -1.81826; tcp_pos.rpy.ry = -0.834253; tcp_pos.rpy.rz = -2.30243; 
    //初始化返回值 
    JointValue joint_pos = { 0,0,0,0,0,0 }; ; 
    demo.login_in("192.168.2.194"); 
    //求逆解 
    ret = demo.kine_inverse(0, &ref_jpos, &tcp_pos, &joint_pos); 
    std::cout << ret << std::endl; 
    for (int i = 0; i < 6; i++) 
    { 
        std::cout << "joint [" << i + 1 << "] is :" << joint_pos.jVal[i] << std::endl; 
    } 
    return 0;
} 
```



#### 机械臂求解正解

计算指定关节位置在当前工具、当前安装角度以及当前用户坐标系设置下的位姿值

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1)
  - joint_pos 关节空间位置
  - cartesian_pose 笛卡尔空间位姿计算结果
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t kine_forward(int robot_id, const JointValue* joint_pos, CartesianPose* cartesian_pose);
```

代码示例：

```C++
#include <iostream> 
#include "JAKAZuRobot.h" 


// 4.55机械臂正解 已知joint_pos,求tcp_pos 
int example_kine_forward() 
{
    int ret; 
    JAKAZuRobot demo; 
    //初始化返回值 
    CartesianPose tcp_pos; 
    demo.login_in("192.168.2.194"); 
    //初始化关节矩阵 
    JointValue joint_pos = { 0.558, 0.872, 0.872 , 0.349, 0.191, 0.191 }; 
    //求正解 
    ret = demo.kine_forward(0, &joint_pos, &tcp_pos); 
    std::cout << ret << std::endl; 
    std::cout << "tcp_pos is :\n x: " << tcp_pos.tran.x << " y: " << tcp_pos.tran.y << " z: " << tcp_pos.tran.z << std::endl; 
    std::cout << "rx: " << tcp_pos.rpy.rx << " ry: " << tcp_pos.rpy.ry << " rz: " << tcp_pos.rpy.rz << std::endl; 
    return 0; 
}  
```



#### 获取两个机械臂的DH参数

获取机器人DH参数

- 参数说明
  - dhParam DH参数
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_get_multi_robot_dh(DHParam *dhParam);
```



#### 设置运动学参数补偿

设置7轴全运动学参数补偿，一组机器人同时生效

- 参数说明
  - flag 1:补偿；0:不补偿
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t set_full_dh_flag(int flag);
```



#### 设置重力方向

设置重力相对于机器人基座的方向，一组机器人同时生效

- 参数说明
  - rpy rpy旋转角；T = rotx(rpy[0]) * roty(rpy[1]) * rotz(rpy[2])
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t set_gravity_direction(const double* flag);
```



#### 获取重力方向

读取重力相对于机器人基座的方向

- 参数说明
  - rpy旋转角；T = rotx(rpy[0]) * roty(rpy[1]) * rotz(rpy[2])
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t get_gravity_direction(const double* flag);
```



### 机械臂力控模块

JAKA机器人提供了一套基于力矩传感器的力控接口，用户可以基于这些接口完成高级的力控功能如导纳控制、柔顺控制等，进而实现一些较复杂的应用场景，如笛卡尔空间指定方向拖拽、力控装配等。但需要注意的是，这些接口依赖于额外配置的工具末端力传感器。

#### 设置机器人末端工具负载

设置机器人末端工具负载

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1)
  - payload 机器人负载参数  2维
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_set_tool_payload(int robot_id, const PayLoad* payload);
```
请在机器人静止状态下进行配置，否则可能会导致运动异常。配置成功后会保存该配置。


#### 获取机器人末端工具负载

获取机器人末端工具负载

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1)
  - payload 机器人负载参数 2维
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_get_tool_payload(int robot_id, const PayLoad* payload);
```



#### 设置传感器灵敏度

设置传感器灵敏度接口

- 参数说明
  - sensor_id 传感器ID
  - deadzone_percent 传感器灵敏度死区占默认最死区的百分比，数据范围0-1
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_set_ftsensor_deadzone(int sensor_id, const double* deadzone_percent);
```



#### 获取传感器灵敏度

获取传感器灵敏度接口

- 参数说明
  - sensor_id 传感器ID
  - deadzone_percent 传感器灵敏度死区占默认最死区的百分比，数据范围0-1
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_get_ftsensor_deadzone(int sensor_id, const double* deadzone_percent);
```



#### 设置力控坐标系接口

设置力控坐标系接口

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1) 
  - ftframe 力控坐标系 接收工具(0) 世界(1)
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_set_cst_ftframe (int robot_id, int ftframe);
```



#### 获取力控坐标系接口

获取力控坐标系接口

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1)
  - ftframe 力控坐标系 接收工具(0) 世界(1)
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_get_cst_ftframe (int robot_id, int ftframe);
```



#### 设置恒力柔顺控制参数获取机器人DH参数

设置恒力柔顺控制参数接口

- 参数说明
  - robot_id  机器人ID 接收LEFT(0) RIGHT(1) 
  - axis 要设置的方向，数据范围0~5，依次代表fx fy fz mx my mz
  - enable 是否开启力控，接收关(0) 开(1)
  - ftUser 力控刚度，需为正数，请勿设置为0，一般建议x,y,z设置为10以上的数值，mx,my,mz设置1以上的数值
  - ftReboundFK 回弹力系数，需为正数，可以设置为0
  - ftConstant 目标力，请勿设置超出机器人承受范围的目标力
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_set_cst_ftconfig(int robot_id, int axis, int enable, double ftUser, double ftReboundFK, double ftConstant);
```

代码示例：

```c++
#include <iostream> 
#include "JAKAZuRobot.h" 


//设置柔顺控制参数 
int example_set_admit_ctrl_config() 
{ 
    int ret; 
    JAKAZuRobot demo; 
    demo.login_in("192.168.2.194"); 
    demo.power_on(); 
    demo.enable_robot(); 
    //设置柔顺控制参数 
    ret = demo.robot_set_cst_ftconfig (0,0,1,15,0,0); 
    std::cout << ret << std::endl; 

  return 0; 

} 
```



#### 获取恒力柔顺控制参数

获取恒力柔顺控制参数接口

- 参数说明
  - robot_id  机器人ID 接收LEFT(0) RIGHT(1) 
  - config 恒力柔顺控制参数
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_get_cst_ftconfig(int robot_id, RobotAdmitCtrl* config);
```



#### 开启恒力柔顺控制接口

开启恒力柔顺控制，恒力柔顺控制可以配合直线运动或关节运动以及笛卡尔伺服运动模式使用，不可以配合拖拽模式或手动示教JOG使用

- 参数说明
  - robot_id  机器人ID 接收LEFT(0) RIGHT(1)
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_enable_force_control(int robot_id);
```



#### 关闭恒力柔顺控制接口

关闭恒力柔顺控制，注意关闭力控将使力控立即失效但不会退出力控模式，直至机器人执行完当前运动（或退出伺服运动模式）后力控模式才会真正退出，因此在此期间无法再次开启力控

- 参数说明
  - robot_id  机器人ID 接收LEFT(0) RIGHT(1) 
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_disable_force_control(int robot_id);
```



#### 设置传感器负载

设置传感器末端负载接口，注意传感器负载与机器人负载可以相同（也可以不同），需要单独设置

- 参数说明
  - sensor_id  传感器ID
  - payload 负载数据 单位kg.m
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_set_ftsensor_payload(int sensor_id, PayLoad* payload);
```
注意：内嵌力传感器的机型不支持使用该接口进行配置，请使用``robot_set_tool_payload``接口进行配置



#### 获取传感器负载

获取传感器末端负载接口，注意传感器负载与机器人负载可以相同（也可以不同），需要单独设置

- 参数说明
  - sensor_id  传感器ID
  - payload 负载数据 单位kg.m
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_get_ftsensor_payload(int sensor_id, PayLoad* payload);
```
注意：内嵌力传感器的机型不支持使用该接口进行读取，请使用``robot_get_tool_payload``接口进行配置


#### 设置传感器数据平滑滤波器截止频率

设置传感器数据平滑滤波器截止频率

- 参数说明
  - sensor_id 传感器ID
  - filter 传截止频率，需为正数，单位Hz
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_set_ftsensor_filter(int sensor_id, double fileter);
```



#### 获取传感器数据平滑滤波器截止频率

获取传感器数据平滑滤波器截止频率

- 参数说明
  - sensor_id 传感器ID
  - filter 传截止频率，需为正数，单位Hz
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_get_ftsensor_filter(int sensor_id, double fileter);
```



#### 获取力控模式状态

获取力控模式状态，注意当调用disable_force_control后但机器人尚未完成当前运动运动期间利空模式仍被视为开启状态

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1)
  - force_control_stat 0代表关闭，1代表开启
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_get_force_control_stat(int robot_id, int* force_control_stat);
```



#### 获取传感器状态

获取传感器状态

- 参数说明
  - sensor_id 传感器ID
  - status 正常(1) 错误(-1)
  - errcode 无数据(1) 数据错误(2) 过载(4)
  - ft_original 传感器在自身坐标系下的原始读数
  - ft_actual 传感器在法兰坐标系下的经过负载和零点补偿后的读数
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_get_ftsensor_stat(int sensor_id, int* status, int* errcode, double* ft_original, double* ft_actual);
```



#### 设置关节的控制环参数

设置关节的控制环参数

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1)
  - joint_id 关节ID
  - pos_kp 位置环比例增益，推荐30，范围0~100，参数越大，位置环越刚，该参数设置过大会导致机器人出现振动，建议在默认值附近调整
  - vel_kp 速度环比例增益，推荐50，范围0~200，参数越大，速度环越刚，该参数设置过大会导致机器人出现振动，建议在默认值附近调整
  - vel_ti 速度环积分常数，推荐2000，范围500~16777216，参数越大，积分作用越强，该参数设置过小会导致速度超调甚至失稳，建议在默认值附近调整
- 返回值 ERR_SUCC 成功 其他失败
- note 当且仅当开启controlloop时才有效。关闭controlloop后，参数会被重置为内部默认值
- note 仅支持3.0.3_ZY1及其以上版本 

```c++
errno_t set_joint_controlloop(int robot_id, int joint_id, uint32_t pos_kp, uint32_t vel_kp, uint32_t vel_ti);
```



#### 开启或关闭关节的控制环参数

开启或关闭关节的控制环参数

- 参数说明
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1)
  - joint_id 关节ID
  - en 是否开启控制环参数
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t enable_joint_controlloop(int robot_id, int joint_id, bool en);
```



#### 传感器校零

传感器校零接口，注意校零需要约1s时间，期间无法开启力控，固建议调用此接口后等待1s。一般建议在开启力控前都进行一次传感器校零，注意校零时传感器不要受到除负载重力以外的任何外力

- 参数说明
  - sensor_id 传感器ID
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_zero_ftsensor(int sensor_id);
```



#### 设置机器人力传感器的软限位

指定机器人配置力传感器的软限位，通过设定阈值用于触发安全保护行为
- 参数说明
  - rules 力传感器的软限位规则
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1)
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_zero_ftsensor(int sensor_id);
```



#### 获取机器人力矩传感器的软限位

查询指定机器人当前配置的力传感器软限位

- 参数说明
  - rules 力传感器的软限位规则
  - robot_id 机器人ID 接收LEFT(0) RIGHT(1)
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t robot_zero_ftsensor(int sensor_id);
```



### FTP服务

#### 初始化FTP客户端

初始化ftp客户端，与控制柜建立连接，可导出program、track

- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t init_ftp_client();
```



#### 初始化加密FTP客户端

初始化ftp客户端，与控制柜建立加密连接，可导出program、track

- 参数说明
  - password 机器人登陆密码
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t init_ftp_client_with_ssl(char* password);
```



#### FTP上传

从本地上传指定类型和名称的文件到控制器

- 参数说明
  - remote 上传到控制器内部文件名绝对路径，若为文件夹需要以“\”或“/”结尾
  - local 本地文件名绝对路径，若为文件夹需要以“\”或“/”结尾
  - opt 1单个文件 2文件
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t upload_file(char* local, char* remote, int opt);
```



#### FTP下载

从控制器下载指定类型和名称的文件到本地

- 参数说明
  - remote 控制器内部文件名绝对路径，若为文件夹需要以“\”或“/”结尾
  - local 下载到本地文件名绝对路径，若为文件夹需要以“\”或“/”结尾
  - opt 1单个文件 2文件夹
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t enable_joint_controlloop(int robot_id, int joint_id, bool en);
```



#### FTP目录查询

查询FTP目录

- 参数说明
  - remote 控制器内部文件名原名称，查询轨迹“/track/”,查询脚本程序“/program/”
  - opt 0文件名和子目录名 1文件名 2子目录名
  - et 返回的查询结果
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t get_ftp_dir(const char* remotedir, int type, char* ret);
```



#### FTP删除

从控制器删除指定类型和名称的文件

- 参数说明
  - remote 控制器内部文件名
  - opt 1单个文件 2文件夹
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t del_ftp_file(char* remote, int opt);
```



#### FTP重命名

重命名控制器指定类型和名称的文件

- 参数说明
  - remote 控制器内部文件名原名称
  - des 重命名的目标名
  - opt 1单个文件 2文件夹
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t ename_ftp_file(char* remote, char* des, int opt);
```



#### 关闭FTP客户端

断开与控制器ftp链接

- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t close_ftp_client();
```



### TIO服务

#### 设置TIO电压参数

设置机器人TIO 的电压参数，仅对硬件版本为 3 的 TIO 生效

- 参数说明
  - robot_id 机器人索引号，0表示左臂，1表示右臂
  - vout_enable 电压输出使能选项，0：关闭，1：开启
  - vout_vol 输出电压设置选项，0：24V
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t set_tio_vout_param(int robot_id, int vout_enable, int vout_vol);
```



#### 获取TIO电压参数

获取cobot机器人TIO 的电压参数，仅对硬件版本为 3 的 TIO 生效

- 参数说明
  - robot_id 机器人索引号，0表示左臂，1表示右臂
  - vout_enable 指针，返回电压输出使能选项，0：关闭，1：开启
  - vout_vol 指针，返回输出电压设置选项，0：24V，1：12V
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t get_tio_vout_param(int robot_id, int* vout_enable, int* vout_vol);
```



#### 设置TIO引脚模式

设置 TIO 引脚模式

- 参数说明
  - robot_id 机器人索引号，0表示左臂，1表示右臂
  - pin_type TIO 类型：0 表示 DI 引脚，1 表示 DO 引脚，2 表示 AI 引脚
  - pin_mode TIO 模式： 
    - DI 引脚：
      - 0: 0x00 DI2 为 NPN，DI1 为 NPN
      - 1: 0x01 DI2 为 NPN，DI1 为 PNP
      - 2: 0x10 DI2 为 PNP，DI1 为 NPN
      - 3: 0x11 DI2 为 PNP，DI1 为 PNP
    - DO 引脚：
      - 低 8 位数据，高 4 位为 DO2 配置，低 4 位为 DO1 配置
      - 0x0 DO 为 NPN 输出
      - 0x1 DO 为 PNP 输出
      - 0x2 DO 为推挽输出
      - 0xF RS485H 接口
    - AI 引脚：
      - 0: 模拟输入功能使能，RS485L 禁用
      - 1: RS485L 接口使能，模拟输入功能禁用
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t set_tio_pin_mode(int robot_id, int pin_type, int pin_mode);
```



#### 获取TIO引脚模式

获取指定类型 TIO 引脚模式

- 参数说明
  - robot_id 机器人索引号，0表示左臂，1表示右臂
  - pin_type TIO 类型：0 表示 DI 引脚，1 表示 DO 引脚，2 表示 AI 引脚
  - pin_mode TIO 模式： 
    - DI 引脚：
      - 0: 0x00 DI2 为 NPN，DI1 为 NPN
      - 1: 0x01 DI2 为 NPN，DI1 为 PNP
      - 2: 0x10 DI2 为 PNP，DI1 为 NPN
      - 3: 0x11 DI2 为 PNP，DI1 为 PNP
    - DO 引脚：
      - 低 8 位数据，高 4 位为 DO2 配置，低 4 位为 DO1 配置
      - 0x0 DO 为 NPN 输出
      - 0x1 DO 为 PNP 输出
      - 0x2 DO 为推挽输出
      - 0xF RS485H 接口
    - AI 引脚：
      - 0: 模拟输入功能使能，RS485L 禁用
      - 1: RS485L 接口使能，模拟输入功能禁用
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t get_tio_pin_mode(int robot_id, int pin_type, int* pin_mode);
```



#### 配置RS485通道通信

配置指定 RS485 通道的通信

- 参数说明
  - robot_id 机器人索引号，0表示左臂，1表示右臂
  - ModRtuComm 通道模式设置为 Modbus RTU 时，需要额外指定 Modbus 从节点 ID
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t set_rs485_chn_comm(int robot_id, ModRtuComm mod_rtu_com);
```



#### 获取RS485通信设置

获取 RS485 通信设置

- 参数说明
  - robot_id 机器人索引号，0表示左臂，1表示右臂
  - ModRtuComm 通道模式设置为 Modbus RTU 时，需要额外指定 Modbus 从节点 ID
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t get_rs485_chn_comm(int robot_id, int chn_id,ModRtuComm* mod_rtu_com);
```



#### 设置RS485通道模式

设置指定 RS485 通道的模式

- 参数说明
  - robot_id 机器人索引号，0表示左臂，1表示右臂
  - chn_id 通道 ID，0 表示 RS485H（通道 1），1 表示 RS485L（通道 2）
  - chn_mode 通道使用模式，0 表示 Modbus RTU，1 表示原始 RS485，2 表示力矩传感器
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t set_rs485_chn_mode(int robot_id, int chn_id, int chn_mode);
```



#### 获取指定RS485通道的模式

获取指定 RS485 通道的模式

- 参数说明
  - robot_id 机器人索引号，0表示左臂，1表示右臂
  - chn_id 通道 ID，0 表示 RS485H（通道 1），1 表示 RS485L（通道 2）
  - chn_mode 通道使用模式，0 表示 Modbus RTU，1 表示原始 RS485，2 表示力矩传感器
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t get_rs485_chn_mode(int robot_id, int chn_id, int* chn_mode);
```



#### 发送指定RS485通道命令

发送指定 RS485 通道的命令

- 参数说明
  - robot_id 机器人索引号，0表示左臂，1表示右臂
  - chn_id TIO 中 RS485 通道的 ID
  - data 命令数据
  - buffsize  data指向的数据长度（字节数），即需要发送的数据大小
- 返回值 ERR_SUCC 成功 其他失败

```c++
errno_t send_tio_rs_command(int robot_id, int chn_id, uint8_t* data,int buffsize);
```





## 接口调用返回值列表及问题排查

| 错误代码 | 描述                                          | 处理建议                                                     |
| -------- | --------------------------------------------- | ------------------------------------------------------------ |
| 0        | success                                       | null                                                         |
| 2        | interface error or controller not support     | 核对控制器和SDK版本信息，进行升级或换用其他接口              |
| -1       | invalid handler                               | 请检查调用接口前是否login                                    |
| -2       | invalid parameter                             | 请检查参数是否正确                                           |
| -3       | fail to connect                               | 请检查网络连接状态，或机器人IP是否正确                       |
| -4       | kine_inverse error                            | 逆解失败，请检查当前的坐标系，或参考关节角是否合理           |
| -5       | e-stop                                        | 急停状态，保留状态                                           |
| -6       | not power on                                  | 未上电                                                       |
| -7       | not enable                                    | 未使能                                                       |
| -8       | not in servo mode                             | 不处于伺服模式，在执行servoJP 的时候，必须先进入伺服模式，请检查是否调用对应接口 |
| -9       | must turn off enable before power off         | 下电前必须下使能                                             |
| -10      | cannot operate, program is running            | 无法操作，程序正在执行中，请先关闭程序                       |
| -11      | cannot open file, or file doesn't exist       | 无法打开文件，或者文件不存在                                 |
| -12      | motion abnormal                               | 运动异常，可能处于奇异点附近，或者超出机器人运动限制         |
| -14      | ftp error                                     | ftp异常                                                      |
| -15      | socket msg or value oversize                  | 超出限制异常                                                 |
| -16      | kine_forward error                            | 正解失败                                                     |
| -17      | not support empty folder                      | 不支持空文件夹                                               |
| -20      | protective stop                               | 保护性停止                                                   |
| -21      | emergency stop                                | 急停                                                         |
| -22      | on soft limit                                 | 处于软限位，此时无法手动拖动机器人，需要用APP上的示教功能接触软限位 |
| -30      | fail to encode cmd string                     | 命令编码失败，一般是解析控制器返回的消息时出错               |
| -31      | fail to decode cmd string                     | 命令解码失败，一般是解析控制器返回的消息时出错               |
| -32      | fail to uncompress port 10004 string          | 解压缩10004端口数据失败，可能受网络波动影响，或数据量太大的原因 |
| -40      | move linear error                             | 直线运动失败，请检查是否路径中是否有奇异区域                 |
| -41      | move joint error                              | 关节运动失败，请检查设置的关节角度                           |
| -42      | move circular error                           | 圆弧运动失败,请检查设置的参数                                |
| -50      | block_wait timeout                            | 阻塞等待超时                                                 |
| -51      | power on timeout                              | 上电超时                                                     |
| -52      | power off timeout                             | 下电超时                                                     |
| -53      | enable timeoff                                | 使能超时                                                     |
| -54      | disable timeout                               | 下使能超时                                                   |
| -55      | set userframe timeout                         | 设置用户坐标系超时                                           |
| -56      | set tool timeout                              | 设置工具坐标系超时                                           |
| -57      | edg init failed                               | edg功能初始化失败                                            |
| -58      | edg is running, cannot use servo_j or servo_p | edg功能正在执行中，无法使用servo_j或者servo_p接口            |
| -60      | set io timeout                                | 设置IO超时                                                   |



## 问题反馈

文档中若出现不准确的描述或者错误，恳请读者见谅指正。如果您在阅读过程中发现任何问题或者有想提出的意见，可以发送邮件到 support@jaka.com ，我们将尽快给您回复。