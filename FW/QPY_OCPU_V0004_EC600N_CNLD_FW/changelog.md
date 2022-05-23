## Release History：
**[QPY_OCPU_V0004_EC600N_CNLD_FW] 2022-04-21**
* ZH
* 1、audio播放路径支持'/usr'方式
* 2、解决外部中断异常抖动触发大量中断导致死机的问题
* 3、解决sys_bus线程和回调中都publish的时候可能会出现死锁的问题
* 4、ntptime支持带入时区参数配置
* 5、新增带密码的交互保护功能
* 6、打开内建方法frozenset及reversed
* 7、解决回调中代码复杂度较高时可能会触发虚拟机assert的问题
* 8、解决设置sms存储位置为ME之后仍旧存储在SM的问题
* 9、修复RTC回调不能正常触发的问题

* EN
* 1.The audio playback path supports the '/usr' method
* 2.Solve the problem that the abnormal jitter of external interrupt triggers a large number of interrupts and causes the crash
* 3.Solve the problem that deadlock may occur when both the sys_bus thread and the callback are published
* 4.ntptime supports bringing in time zone parameter configuration
* 5.Added interactive protection with password
* 6.Open built-in methods frozenset and reversed
* 7.Solve the problem that the virtual machine assert may be triggered when the code complexity in the callback is high
* 8.Solve the problem that the sms storage location is still stored in SM after setting it to ME
* 9.Fix the problem that the RTC callback cannot be triggered normally



**[QPY_OCPU_V0003_EC600N_CNLD_FW] 2022-01-20**
1、解决部分硬件无法烧录的问题
2、解决打开debug口后再实例化Pin，会触发死机的问题



**[QPY_OCPU_V0002_EC600N_CNLD_FW] 2022-01-11**

1、解决将bound meth设置为回调后，若回调高频触发后导致dump的问题
2、添加播放tone音接口
3、amr录音新增dtx控制开关
4、新增模拟IIC功能
5、queciot版本升级至2.9.0
6、解决因实现CTRL_C功能导致input()无法使用的问题
7、sys_bus支持取消订阅功能
8、解决SPI传输过程中遇0x00停止传输的问题
9、创建线程新增参数使能线程异常自恢复功能
10、支持W5500有线网卡
11、解决未注网状态下执行c=utime.mktime(utime.localtime()) utime.localtime(c)抛异常的问题
12、mqtt修复在重连成功后，服务器马上回复信息&自动订阅topic产生冲突，导致一直报错，重连
13、解决utime.ticks_ms在计数到0x7fffffff之后转变为非常大的数值的问题
14、解决多路拨号拨号接口阻塞5分钟的问题
15、升级基线解决VOLTE在部分场景电话无声的问题（EC600NCN_LC）
16、解决open debug口之后，debug口仍有log吐出的问题
17、解决因中断引脚电平抖动，导致gc内存耗尽触发死机问题
19、解决录音流触发回调过多，导致gc内存耗尽触发死机问题
20、解决TTS播放Unicode编码无法播放数字和字母的问题


**[QPY_OCPU_V0001_EC600N_CNLD_FW] 2021-11-17**

* 1、新框架基础功能:LCD、VOLTE、SPISD、CAMERA、LVGL等基础功能支持
* 2、新增keypad功能
* 3、新增WTTS功能
* 4、DM9051以太网卡
* 5、支持control_485
* 6、新增功能支持CRTLC




