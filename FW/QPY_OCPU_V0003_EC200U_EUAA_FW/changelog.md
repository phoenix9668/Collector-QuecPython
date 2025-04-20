## Release History：
**[QPY_OCPU_V0003_EC200U_EUAA_FW] 2023-06-14**

* ZH
* 1、解决获取不到IPV6地址的问题
* 2、解决关闭热插拔功能时返回-1的问题
* 3、修复net.setApn;dataCall.setPDPContext设置用户名密码为空字符串时，设置后不生效问题
* 4、修复初始化网卡后，重新拨号后，DNS解析失败问题
* 5、修复GSM网络下出现net.getSignal查询rssi=0的情况
* 6、修复执行recv过程中，close socket导致dump的问题



* EN
* 1. Solve the problem that the IPV6 address cannot be obtained
* 2. Resolved the problem that -1 is returned when the hot swap function is disabled
* 3. Fixed net.setApn; dataCall.setPDPContext Does not take effect after the user name and password are set to empty strings
* 4. Fixed DNS resolution failure after NIC initialization and dial-up again
* 5. Fix the situation in which net.getSignal queries rssi=0 on the GSM network
* 6. Fixed an issue where the socket was closed during recv execution, causing dump




**[QPY_OCPU_V0002_EC200U_EUAA_FW] 2023-03-10**

* ZH
* 1、解决低功耗模式耗流值异常问题
* 2、添加一路串口UART4
* 3、解决 USBNET 功能开启后无法使用网络问题
* 4、解决MIPI屏在拔掉USB后会花屏的问题
* 5、解决MIPI rgb_order参数不生效问题
* 6、修复SD卡初始化失败后再次初始化阻塞20s的问题
* 7、解决流播内存泄漏问题
* 8、当net_mode和net_status引脚作为GPIO使用时，可以禁止网络灯功能
* 9、增加通过4线SPI挂载NOR Flash作为文件系统的功能
* 10、修复wifiScan扫描热点超过30个时出现dump或者卡住的问题
* 11、解决流播时，AMR不传头文件播放失败的问题
* 12、解决TTS播放接口会阻塞160ms左右的问题
* 13、解决使用atcmd模块接口发送AT指令，概率性dump问题
* 14、ble功能模块增加ibeacon相关接口
* 15、ble功能模块增加修改特征值接口
* 16、解决SPI CS管脚初始化失败问题
* 17、修复开启USBNET功能后，获取的IP有误的问题
* 18、修复删除或格式化SD卡文件出现异常失败的问题
* 19、解决关机时执行RTC同步导致关机闹钟概率性失效问题
* 20、解决UART回调导致的dump问题
* 21、添加参数用于控制MIPI RST极性
* 22、用户可配置开pa时发送的脉冲数
* 23、socket功能优化
* 24、增加webserver功能支持
* 25、修复queue可能导致死锁的问题
* 26、USBNET模块增加NAT设置和查询功能
* 27、增加uPing和uwebsocket功能
* 28、增加SPI 驱动LCD的接口
* 29、外挂GNSS数据解析模块性能优化
* 30、checkNet功能模块代码优化
* 31、request模块增加对HTTP1.1版本支持
* 32、阿里云增加对ssl连接的支持
* 33、machine lcd模块增加ISINK背光控制引脚背光挡位控制接口lcd_level_brightness
* 34、经典蓝牙功能模块bt增加重连功能设置接口
* 35、修复ussl 双认证密钥错误导致握手失败后重启虚拟机导致dump的问题
* 36、ntptime模块对时接口settime增加一个参数use_rhost让用户设置是否使用备用NTP服务器地址
* 37、machine uart 模块新增read_once()/write_once()接口
* 38、umqtt重连功能优化
* 39、解决流式录音会阻塞其他python线程运行的问题

* EN
* 1. Solve the problem of abnormal current consumption value in low power mode
* 2. Add one serial port UART4
* 3. Solve the problem that the network cannot be used after USBNET function is enabled
* 4, solve the MIPI screen after unplugging the USB screen will spend the problem
* 5. Resolve the problem that MIPI rgb_order does not take effect
* 6. Fixed SD card initialization blocking 20s again after failed initialization
* 7. Solve the problem of streaming memory leakage
* 8. Disable network light when the net_mode and net_status pins are used as GPIO
* 9. Added the ability to mount NOR Flash as a file system via 4-wire SPI
* 10. Fix dump or jam when wifiScan scans more than 30 hot spots
* 11. Solve the problem that AMR cannot play header files when streaming
* 12. Solve the problem that the TTS playback interface will block around 160ms
* 13. Solve the probabilistic dump problem of sending AT commands using the atcmd interface
* 14. Add ibeacon related interfaces to ble function module
* 15. Added an interface for modifying feature values in the ble function module
* 16. Solve the SPI CS pin initialization failure problem
* 17. Fix the incorrect IP address obtained after USBNET is enabled
* 18. Fix failure to delete or format SD card files abnormally
* 19. Solve the problem that the shutdown alarm clock may fail due to RTC synchronization during shutdown
* 20. Resolve the dump problem caused by the UART callback
* 21. Add parameters to control MIPI RST polarity
* 22. The user can configure the number of pulses sent when pa is on
* 23. socket function optimization
* 24. Added support for webserver functions
* 25. Fixed an issue where queue could cause deadlock
* 26. Add NAT Settings and query functions in USBNET module
* 27. Added uPing and uwebsocket functions
* 28. Add SPI driver LCD interface
* 29. Performance optimization of external GNSS data parsing module
* 30, checkNet function module code optimization
* 31. Added support for HTTP1.1 in request module
* 32. Aliyun adds support for ssl connections
* 33. machine lcd module added ISINK backlight control pin backlight gear control interface lcd_level_brightness
* 34. The classic Bluetooth function module bt adds the reconnection function setting interface
* 35. Fix the dump problem caused by VM restart after handshake failure due to incorrect ussl dual authentication key
* 36. Add a parameter use_rhost to the settime interface of the ntptime module to enable users to set whether to use the address of the standby NTP server
* 37. Added the read_once()/write_once() interface to the machine uart module
* 38. umqtt reconnection function optimization
* 39. Resolve the problem that streaming recording blocks other python threads

------

**[QPY_OCPU_V0001_EC200U_EUAB_FW] 2022-07-07**

* ZH
* 1、支持QuecPython类库voiceCall - 电话功能（仅2G模式下可打电话）
* 2、支持QuecPython标准库uos - 基本系统服务组件功能
* 3、支持QuecPython标准库gc - 内存碎片回收功能
* 4、支持QuecPython标准库ubinascii - 二进制与ASCII转换功能
* 5、支持QuecPython标准库ucollections - 集合和容器类型功能
* 6、支持QuecPython标准库urandom - 生成随机数功能
* 7、支持QuecPython标准库math - 数学运算功能
* 8、支持QuecPython标准库usocket - socket通信功能
* 9、支持QuecPython标准库uio - 输入输出流功能
* 10、支持QuecPython标准库ustruct - 打包和解压原始数据类型功能
* 11、支持QuecPython标准库ujson - JSON编码和解码功能
* 12、支持QuecPython标准库utime - 时间相关功能
* 13、支持QuecPython标准库sys/usys - 系统相关功能
* 14、支持QuecPython标准库uzlib - zlib解压缩功能
* 15、支持QuecPython标准库_thread - 多线程功能
* 16、支持QuecPython标准库uhashlib - 哈希算法功能
* 17、支持QuecPython类库example - 执行python脚本功能
* 18、支持QuecPython类库dataCall - 数据拨号功能
* 19、支持QuecPython类库cellLocator - 基站定位功能
* 20、支持QuecPython类库wifilocator - wifi定位功能
* 21、支持QuecPython类库sim - SIM卡相关功能
* 22、支持QuecPython类库sms - 短信功能
* 23、支持QuecPython类库net - 网络相关功能
* 24、支持QuecPython类库checkNet - 等待网络就绪
* 25、支持QuecPython类库fota - 固件升级功能
* 26、支持QuecPython类库app_fota - 用户文件升级功能
* 27、支持QuecPython类库audio - 音频播放功能
* 28、支持QuecPython类库record - 录音功能
* 29、支持QuecPython类库misc - Power相关功能
* 30、支持QuecPython类库misc - PowerKey相关功能
* 31、支持QuecPython类库misc - PWM功能
* 32、支持QuecPython类库misc - ADC功能
* 33、支持QuecPython类库misc - USB插拔检测
* 34、支持QuecPython类库misc - USBNET功能
* 35、支持QuecPython类库modem - 获取设备信息相关功能
* 36、支持QuecPython类库machine - Pin功能
* 37、支持QuecPython类库machine - UART功能
* 38、支持QuecPython类库machine - Timer功能
* 39、支持QuecPython类库machine - ExtInt中断
* 40、支持QuecPython类库machine - RTC功能
* 41、支持QuecPython类库machine - I2C功能
* 42、支持QuecPython类库machine - I2C_simulation功能
* 43、支持QuecPython类库machine - SPI功能
* 44、支持QuecPython类库machine - LCD功能/MIPI
* 45、支持QuecPython类库machine - WDT功能
* 46、支持QuecPython类库machine - KeyPad功能
* 47、支持QuecPython类库pm - 低功耗功能
* 48、支持QuecPython类库ure - 正则功能
* 49、支持QuecPython类库wifiScan - wifi热点扫描功能
* 50、支持QuecPython类库gnss - 外接gnss(L76K)定位数据解析
* 51、支持QuecPython类库camera - 摄像头功能（可选功能）
* 52、支持QuecPython类库qrcode - 二维码显示（可选功能）
* 53、支持QuecPython第三方库aLiYun - 阿里云服务
* 54、支持QuecPython第三方库TenCentYun- 腾讯云服务
* 55、支持QuecPython第三方库request - HTTP
* 56、支持QuecPython第三方库log - 日志
* 57、支持QuecPython第三方库umqtt - MQTT
* 58、支持QuecPython第三方库ntptime - NTP对时
* 59、支持QuecPython第三方库system - 环境配置
* 60、支持QuecPython第三方库ql_fs - 高级文件操作
* 61、支持QuecPython第三方库Queue - 消息队列
* 62、支持QuecPython第三方库sys_bus - 会话总线
* 63、支持QuecPython第三方库uasyncio - 协程
* 64、支持QuecPython第三方库ssl - 加密算法
* 65、支持QuecPython类库ble - 低功耗蓝牙
* 66、支持QuecPython类库bt - 经典蓝牙（做从）

* EN
* 1. Support QuecPython class library voiceCall - phone function (only 2G mode can make calls)
* 2. Support QuecPython standard library uos - basic system service component function
* 3. Support QuecPython standard library gc - memory fragmentation recovery function
* 4. Support QuecPython standard library ubinascii - binary and ASCII conversion function
* 5. Support QuecPython standard library ucollections - collection and container type functions
* 6. Support QuecPython standard library urandom - generate random number function
* 7. Support QuecPython standard library math - math operation function
* 8. Support QuecPython standard library usocket - socket communication function
* 9. Support QuecPython standard library uio - input and output stream function
* 10. Support QuecPython standard library ustruct - function of packing and unpacking primitive data types
* 11. Support QuecPython standard library ujson - JSON encoding and decoding functions
* 12. Support QuecPython standard library utime - time-related functions
* 13. Support QuecPython standard library sys/usys - system related functions
* 14. Support QuecPython standard library uzlib - zlib decompression function
* 15. Support QuecPython standard library _thread - multi-threading function
* 16. Support QuecPython standard library uhashlib - hash algorithm function
* 17. Support QuecPython class library example - execute python script function
* 18. Support QuecPython class library dataCall - data dialing function
* 19. Support QuecPython class library cellLocator - base station positioning function
* 20. Support QuecPython class library wifilocator - wifi positioning function
* 21. Support QuecPython class library sim - SIM card related functions
* 22. Support QuecPython class library sms - SMS function
* 23. Support QuecPython class library net - network related functions
* 24. Support QuecPython class library checkNet - wait for the network to be ready
* 25. Support QuecPython class library fota - firmware upgrade function
* 26. Support QuecPython class library app_fota - user file upgrade function
* 27. Support QuecPython class library audio - audio playback function
* 28. Support QuecPython class library record - recording function
* 29. Support QuecPython class library misc - Power related functions
* 30. Support QuecPython class library misc - PowerKey related functions
* 31. Support QuecPython class library misc - PWM function
* 32. Support QuecPython class library misc - ADC function
* 33. Support QuecPython class library misc - USB plug-in detection
* 34. Support QuecPython class library misc - USBNET function
* 35. Support QuecPython class library modem - get device information related functions
* 36. Support QuecPython class library machine - Pin function
* 37. Support QuecPython class library machine - UART function
* 38. Support QuecPython class library machine - Timer function
* 39. Support QuecPython class library machine - ExtInt interrupt
* 40. Support QuecPython class library machine - RTC function
* 41. Support QuecPython class library machine - I2C function
* 42. Support QuecPython class library machine - I2C_simulation function
* 43. Support QuecPython class library machine - SPI function
* 44. Support QuecPython class library machine - LCD function/MIPI
* 45. Support QuecPython class library machine-WDT function
* 46. Support QuecPython class library machine-KeyPad function
* 47. Support QuecPython class library pm - low power consumption function
* 48. Support QuecPython class library ure - regular function
* 49. Support QuecPython class library wifiScan - wifi hotspot scanning function
* 50. Support QuecPython class library gnss - external gnss (L76K) positioning data analysis
* 51. Support QuecPython class library camera - camera function (optional function)
* 52. Support QuecPython class library qrcode - two-dimensional code display (optional function)
* 53. Support QuecPython third-party library aLiYun - Alibaba Cloud Service
* 54. Support QuecPython third-party library TenCentYun- Tencent Cloud Service
* 55. Support QuecPython third-party library request - HTTP
* 56. Support QuecPython third-party library log - log
* 57. Support QuecPython third-party library umqtt - MQTT
* 58. Support QuecPython third-party library ntptime - NTP time synchronization
* 59. Support QuecPython third-party library system - environment configuration
* 60. Support QuecPython third-party library ql_fs - advanced file operations
* 61. Support QuecPython third-party library Queue - message queue
* 62. Support QuecPython third-party library sys_bus - session bus
* 63. Support QuecPython third-party library uasyncio - coroutine
* 64. Support QuecPython third-party library ssl - encryption algorithm
* 65. Support QuecPython class library ble - low energy bluetooth
* 66. Support QuecPython class library bt - classic bluetooth (do slave)
