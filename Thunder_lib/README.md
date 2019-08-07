# 贝尔（bell）雷霆（thunder）Arduino API

>  目录结构
1. 所有源码文件放到Thunder_lib文件夹；
2. Thunder_lib/doc 存放相关规格书、说明文档；
3. Thunder_lib/example 存放Arduino sketch例程；

> 命名规则：
1. 文件名全部字母小写，以下划线分割；类型名全部字母大写，以下划线分割；全局变量名首字母大写，以下划线分割；局部变量全部字母小写；
2. 语义命名规则下，从左到右是大类别到小类别：例如 SENSOR_TOUCH 左边是指示传感器，右边指示触碰传感器，传感器这个类别比触碰传感器这个类别大，所以SENSOR在左，TOUCH在右。MOTOR_FAN 左边MOTOR是电机类别，右边FAN将电机类别缩小为风扇电机
3. 前缀bell、后缀thunder 一般用于标识该产品为 贝尔(bell) 雷霆系列(thunder)
