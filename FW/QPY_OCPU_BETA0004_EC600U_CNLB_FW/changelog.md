## Release History：
**[QPY_OCPU_BETA0004_EC600U_CNLB_FW] 2024-03-06**

* ZN
* 1、合入驱动修改点
* 2、设置展锐8910平台PCM回环时为全双工POC模式
* 3、增加cam_vdd输出可以设定休眠时是否继续输出
* 4、增加aud_set_calib_real_time接口
* 5、增加8910平台 cam_vdd 输出指定的电压 支持1V8 2V5 2V8 3V0
* 6、EC600U添加4线flash型号W25Q64JVSIQ
* 7、展锐R03客户需求W5500，测试并优化移植后的代码



* EN
* 1. Added the drive modification point
* 2. Set PCM loop to full duplex POC mode
* 3. Enabled cam_vdd output can set whether to continue output during sleep
* 4. Added the aud_set_calib_real_time interface
* 5. Added 8910 platform cam_vdd output specified voltage support 1V8 2V5 2V8 3V0
* 6. Added EC600U 4-wire flash model W25Q64JVSIQ
* 7. Enabled W5500, tested and optimized the code after transplantation

## Release History：
**[QPY_OCPU_V0003_EC600U_CNLB_FW] 2023-06-08**

* ZN
* 1、优化CH395网络速率
* 2、新增dhcp server功能
* 3、修复USBnet开启之后，实网IP获取显示错误的问题
* 4、增加耳机检测机制（仅支持EC600U）
* 5、解决烧录版本后，首次以默认ECM模式open后，usbnet.open返回值不正确的问题
* 6、解决camera预览异常问题
* 7、解决SD卡格式化之后创建和之前相同name的文件失败的问题
* 8、解决关机闹钟概率性失效问题
* 9、解决获取不到IPV6地址的问题
* 10、解决关闭热插拔功能时返回-1的问题
* 11、修复net.setApn;dataCall.setPDPContext设置用户名密码为空字符串时，设置后不生效问题
* 12、修复初始化网卡后，重新拨号后，DNS解析失败问题
* 13、修复GSM网络下出现net.getSignal查询rssi=0的情况
* 14、修复执行recv过程中，close socket导致dump的问题
* 15、解决使用atcmd模块接口发送AT概率性发生dump问题
* 16、修复SD卡不close直接remove导致空间泄露的问题


* EN
* 1. Optimize the network rate of the CH395
* 2. Added the dhcp server function
* 3. Fixed an error in obtaining real network IP after USBnet is enabled
* 4. Added headset detection mechanism (EC600U only)
* 5. Fixed an issue where the usbnet.open return value was incorrect after the first open in default ECM mode after burning the version
* 6. Resolved the camera preview exception
* 7. Resolved an issue where a file with the same name as before failed to be created after SD card formatting
* 8. Resolve the problem that the shutdown alarm is likely to fail
* 9. Solve the problem that the IPV6 address cannot be obtained
* 10. Resolved the problem that -1 is returned when the hot swap function is disabled
* 11. Fixed net.setApn; dataCall.setPDPContext Does not take effect after the user name and password are set to empty strings
* 12. Fixed DNS resolution failure after NIC initialization and dial-up again
* 13. Fix the situation in which net.getSignal queries rssi=0 on the GSM network
* 14. Fixed an issue where the socket was closed during recv execution, causing dump
* 15. Solve the problem that probability dump occurs when atcmd is used to send ats
* 16. Fixed space leakage caused by removing SD card without close


**[QPY_OCPU_V0002_EC600U_CNLB_FW] 2022-12-14**

* ZN
* 1、增加一路串口UART4
* 2、USBNET功能优化
* 3、解决重复初始化LCD出现黑屏无响应的问题
* 4、解决MIPI屏在拔掉USB后会花屏的问题
* 5、添加对CH395Q网卡的支持
* 6、修复流播内存泄漏问题
* 7、当net_mode和net_status引脚作为GPIO使用时，可以禁止网络灯功能
* 8、增加通过4线SPI挂载NOR Flash作为文件系统的功能
* 9、修复wifiScan扫描热点超过30个时出现dump或者卡住的问题
* 10、经典蓝牙增加自动重连功能（蓝牙功能需定制版才支持）
* 11、解决流播时，AMR不传头文件播放失败的问题
* 12、TTS播放接口性能优化
* 13、socket功能优化
* 14、增加webserver功能支持
* 15、解决LVGL运行时，按CTRL+D重启虚拟机导致dump的问题
* 16、解决LVGL影响休眠的问题
* 17、修复queue可能导致死锁的问题
* 18、增加NAT设置和查询功能
* 19、增加uPing和uwebsocket功能
* 20、增加SPI驱动LCD接口
* 21、外挂GNSS数据解析模块性能优化
* 22、checkNet功能模块代码优化
* 23、request模块增加对HTTP1.1版本支持
* 24、umqtt重连功能优化

* EN
* 1. Add a serial port UART4
* 2. USBNET function optimization
* 3. Solve the problem of black screen and no response after repeated initialization of LCD
* 4. Solve the problem that the MIPI screen will be blurred after unplugging the USB
* 5. Add support for CH395Q network card
* 6. Fix streaming memory leak problem
* 7. When the net_mode and net_status pins are used as GPIO, the network light function can be disabled
* 8. Added the function of mounting NOR Flash as a file system via 4-wire SPI
* 9. Fix the dump or stuck problem when wifiScan scans more than 30 hotspots
* 10. Classic Bluetooth adds automatic reconnection function (Bluetooth function needs customized version to support)
* 11. Solve the problem that AMR fails to play the header file when streaming
* 12. TTS playback interface performance optimization
* 13. Socket function optimization
* 14. Add webserver function support
* 15. Solve the problem of dump caused by pressing CTRL+D to restart the virtual machine when LVGL is running
* 16. Solve the problem that LVGL affects sleep
* 17. Fix the problem that queue may cause deadlock
* 18. Add NAT setting and query function
* 19. Add uPing and uwebsocket functions
* 20. Add SPI driver LCD interface
* 21. Performance optimization of external GNSS data analysis module
* 22. Code optimization of checkNet function module
* 23. The request module adds support for HTTP1.1 version
* 24. Optimization of umqtt reconnection function



**[QPY_OCPU_V0001_EC600U_CNLB_FW] 2022-07-12**

* ZH
* 1、支持QuecPython第三方库ssl - 加密算法
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
* 50、支持QuecPython类库TTS
* 51、支持QuecPython类库gnss - 外接gnss(L76K)定位数据解析
* 52、支持QuecPython类库camera - 摄像头功能（可选功能）
* 53、支持QuecPython类库qrcode - 二维码显示（可选功能）
* 54、支持QuecPython第三方库aLiYun - 阿里云服务
* 55、支持QuecPython第三方库TenCentYun- 腾讯云服务
* 56、支持QuecPython第三方库request - HTTP
* 57、支持QuecPython第三方库log - 日志
* 58、支持QuecPython第三方库umqtt - MQTT
* 59、支持QuecPython第三方库ntptime - NTP对时
* 60、支持QuecPython第三方库system - 环境配置
* 61、支持QuecPython第三方库ql_fs - 高级文件操作
* 62、支持QuecPython第三方库Queue - 消息队列
* 63、支持QuecPython第三方库sys_bus - 会话总线
* 64、支持QuecPython第三方库uasyncio - 协程

* EN
* 1. Support QuecPython third-party library ssl - encryption algorithm
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
* 50. Support QuecPython class library TTS
* 51. Support QuecPython class library gnss - external gnss (L76K) positioning data analysis
* 52. Support QuecPython class library camera - camera function (optional function)
* 53. Support QuecPython class library qrcode - two-dimensional code display (optional function)
* 54. Support QuecPython third-party library aLiYun - Alibaba Cloud Service
* 55. Support QuecPython third-party library TenCentYun- Tencent Cloud Service
* 56. Support QuecPython third-party library request - HTTP
* 57. Support QuecPython third-party library log - log
* 58. Support QuecPython third-party library umqtt - MQTT
* 59. Support QuecPython third-party library ntptime - NTP time synchronization
* 60. Support QuecPython third-party library system - environment configuration
* 61. Support QuecPython third-party library ql_fs - advanced file operations
* 62. Support QuecPython third-party library Queue - message queue
* 63. Support QuecPython third-party library sys_bus - session bus
* 64. Support QuecPython third-party library uasyncio - coroutine
