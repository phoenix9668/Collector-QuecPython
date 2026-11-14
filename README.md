# Collector-QuecPython

国内 EC600M Collector 固件，运行于 QuecPython，连接阿里云物联网平台。当前应用版本为 `4.1.5`。

## 主要能力

- UART2 `115200 8N1` 高吞吐接收，B0体征包固定206字节；回调只搬运字节，解析、Flash持久化和MQTT发布相互隔离。
- 体征数据采用RAM队列、动态Flash循环队列和阿里云属性上报回复确认，提供有容量边界的至少一次投递。
- DeviceSecret直接认证，或ProductSecret动态注册；按身份永久保留已取得的DeviceSecret，连接失败不删除密钥，使用CRC校验和双份快照恢复；仓库不保存生产密钥。
- 阿里云应用OTA：多文件 `app_fota` HTTP升级，以及兼容旧平台的MQTT分块单 `main.py` 升级。
- 针对实机约576 KiB的 `/usr` 分区实际预占256 KiB OTA空间；体征Flash循环队列动态分配、最多96 KiB，且不会挤占OTA预留、安全余量和运行元数据空间。
- `runtimeLog`、`debugLog`结构化云日志使用独立限速线程和阿里云事件回复确认，本地严重日志双文件合计最多16 KiB。
- `product_information:UartSampleLog`可远程开启低频UART原始数据预览，默认关闭并保存到 `/usr/collector_settings.json`；统计和错误日志不受开关影响。
- 支持通过单设备私有OTA命令迁移ProductKey/DeviceName；迁移期间全部业务采集和上报保持停止，目标身份先动态注册和事件探测，再原子切换，失败自动恢复旧身份。

## 目录

- `src/main.py`：稳定启动和失败回滚入口。
- `src/collector_*.py`：配置、UART、队列、云连接、OTA、日志和传感器模块。
- `src/device.example.json`：设备配置模板。
- `src/device_migration.example.json`：远程身份迁移命令模板；生产副本含ProductSecret，禁止提交。
- `doc/model.zip`：可直接导入阿里云的物模型；`doc/model/`为可编辑源文件。
- `doc/Hardware Protocol-Collector Hardware Send Protocol.png`：Collector硬件通信协议图。
- `fw/QPY_OCPU_EC600M_CNLE_FW_V0006/`：当前EC600M-CN-LE V0606固件及版本说明，根目录同时保留完整压缩包。
- `tools/build_ota_package.py`：生成多文件及旧版单文件OTA制品和MD5清单。
- `tests/`：主机协议、队列、OTA、云确认及物模型测试。

## 首次部署

1. 首次部署下载 `src/main.py` 和全部 `src/collector_*.py` 到 `/usr/`。576 KiB模组的云连接、配置、迁移三个模块须使用 `dist/collector_app_4.1.5/` 中的压缩制品，去掉文件名末尾的 `.bin` 后下载，留足凭证和运行空间。已有4.1.4设备使用下面的三文件OTA包。
2. 以 `src/device.example.json` 为模板创建 `/usr/device.json`：填写ProductKey、DeviceName、MQTT实例地址，并填写DeviceSecret；若使用动态注册则只填写ProductSecret。密钥保存在 `/usr/device_secret.json` 及 `.bak`，写入中可能存在 `.tmp`；每份不超过4 KiB。历史身份及迁移失败目标的密钥均保留，档案满时拒绝新增注册，不淘汰旧记录。
3. 将 `doc/model.zip` 导入对应阿里云产品物模型。
4. 重启设备。设备会先建立256 KiB的 `/usr/.ota_space.reserve`，随后按实际剩余空间创建体征Flash队列；启动日志中的 `flash_frames` 是该设备本次启动的真实容量。

使用QPYcom的 `example.exec('/usr/main.py')` 手动启动也受支持；启动器会主动切换到 `/usr` 并加入模块搜索路径。

`SendCommand`仅为兼容保留。设备收到设置请求会回复成功并记录runtimeLog，但不会向UART发送任何内容。

`runtimeLog`的数值字段使用 `metricValue`。QH-D200模型中的原字段名 `value` 是阿里云物模型保留标识符，不能用于快速导入；构建脚本会在打包前检查全部自定义标识符。

## 构建与测试

```powershell
python -m unittest discover -s tests -v
python tools/build_model_zip.py
python tools/build_ota_package.py --base-version 4.1.4 --target-version 4.1.5 --include collector_cloud.py --include collector_config.py --include collector_migration.py --compact --output dist/collector_app_4.1.5
```

4.1.5的三个模块必须一起升级。`--compact`仅压缩缩进和注释，保留字符串、语法树和源码行号；仓库源码保持可读。升级空间预算为212 KiB，小于256 KiB预留。历史版本应使用已保留的制品，不要用当前源码覆盖构建旧版本包。审查结果见 [4.1.5凭证修复与审查报告](doc/4.1.5凭证修复与审查报告.md)，设备BW-XC-200-002的操作步骤见 [4.1.5单台验收](doc/4.1.5单台验收.md)。

QuecLocator继续使用原EC600M代码内置的服务器、端口、应用令牌和定位参数，不需要在 `device.json` 中配置。

物模型脚本会校验 `doc/model/*.json` 并重建 `doc/model.zip`。OTA构建按每个文件4 KiB、目录8 KiB计入占用，并在清单写入暂存、回滚、updater和安全余量的总需求。576 KiB实机无法一次容纳4.0.13到4.1.0的全部暂存和回滚副本，因此必须依次安装已生成的4.0.15桥接包和4.1.0能力包。阿里云界面仍选择“整包升级”，上传清单列出的全部 `.bin`，并把 `aliyun_custom_info.txt` 原样粘贴到自定义信息。详细步骤见 `doc/EC600M部署与验收.md`。

## 投递边界

正常运行时，在设备持续供电、UART硬件正常且积压不超过启动日志公布的RAM+Flash容量的前提下，程序不会静默丢包。Flash数据可跨重启恢复；仅存在于RAM的数据无法抵御突然断电。回复丢失会使用相同消息ID重试，因此服务端可能收到重复数据，应按设备和消息ID幂等处理。

`SignsData` 使用 MQTT QoS 0 作为非阻塞传输层，并以阿里云 Alink `property/post_reply` 作为端到端确认：只有相同消息ID返回 `code=200` 且 `data` 为空才从队列删除；字段级错误、超时和断线都会保留原消息ID重试。正常发送限速为25条/秒，为阿里云单设备30条/秒上行限额保留余量。

从修复版4.1.2开始，任何被接受的新版本OTA任务都会立即进入独占模式：停止UART解析、传感器和业务上报，清空RAM、Flash及在途SignsData，并在升级期间丢弃新串口数据；只保留MQTT连接、OTA下载和进度上报。升级失败后恢复空队列运行，升级成功则重启。此处是明确的主动丢数边界。

外部看门狗心跳不属于被停止的业务：OTA和身份迁移期间即使MQTT临时断开，也继续每120秒通过UART2发送 `Heartbeat`，避免下位机把受控维护误判为模组故障并复位。正常运行仍沿用原逻辑，只在云连接正常时发送心跳。

在576 KiB分区中，完整256 KiB OTA预留优先于Flash体征队列。应用增长后若没有剩余空间建立Flash队列，设备以512帧RAM队列继续工作并输出一次信息日志；这不再被误报为OTA健康恢复失败。

设备身份迁移同样不再使用序号水位线、25%容量门限或旧队列排空。迁移包重启后业务继续保持停止，优先复用目标历史凭证；仅无凭证时预注册，持久保存后才探测和切换身份。目标runtimeLog确认且连接稳定120秒后再次重启并恢复正常业务，失败则恢复旧身份后重启。恢复写入失败会保留状态并重试；启动时恢复失败会中止应用，等待存储修复和重启。早期4.1.1的精简版`str`兼容问题和历史升级顺序见部署文档。
