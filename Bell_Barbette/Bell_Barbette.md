# 炮台模块库 #

----------
## 描述 ##

        本库文件适用于炮台模块，基于I2C总线控制炮台模块的一系列功能，包括炮台的发射、供弹、俯仰、获取炮台寿命、获取子弹数量等。

----------


## 函数列表 ##

----------


**炮台初始化**

描述：初始化炮台模块

输入：NONE

输出：NONE

	void Bell_Barbette::Begin()

----------


**炮台发射**

描述：以最大发射速度发射子弹

输入：NONE

输出：NONE

	void Bell_Barbette::Open_Fire()

----------
**炮台停止发射**

描述：停止发射子弹

输入：NONE

输出：NONE

	void Bell_Barbette::Stop_Fire()

----------
**改变炮台发射速度**

描述：以一定的速度发射子弹

输入：`rate`：子弹发射速度（0~255）

输出：NONE
	
	void Bell_Barbette::Fire_With_Rate(uint8_t rate)

----------
**开启供弹电机**

描述：以最大速度供弹

输入：NONE

输出：NONE

	void Bell_Barbette::Supply_On()

----------
**停止供弹**

描述：停止供弹

输入：NONE

输出：NONE

	void Bell_Barbette::Supply_Off()

----------
**改变供弹速度**

描述：以一定速度供弹

输入：`rate`：供弹速度（0~255）

输出：NONE
	
	void Bell_Barbette::Supply_With_Rate(uint8_t rate)

----------

**获取已发射子弹数量**

描述：获取炮台已发射子弹数量

输入：NONE

输出：已发射子弹数量

	uint16_t Bell_Barbette::Get_Bullet()

----------
**获取炮台已使用时间**

描述：获取炮台已使用时间

输入：NONE

输出：炮台使用的时间，单位秒

	uint16_t Bell_Barbette::Get_Used_Time()

----------
**修改炮台角度**

描述：修改舵机的俯仰角度

输入：`pos`：炮台的俯仰角度（0~180）

输出：NONE

	void Bell_Barbette::Set_Battery_Pos(uint8_t pos)