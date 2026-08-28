# Collector-QuecPython

国内 EC600M Collector 固件，运行于 QuecPython，连接阿里云物联网平台。当前应用版本为 `4.0.1`。

## 主要能力

- UART2 `115200 8N1` 高吞吐接收，B0体征包固定206字节；回调只搬运字节，解析、Flash持久化和MQTT发布相互隔离。
- 体征数据采用RAM队列、动态Flash循环队列和阿里云属性上报回复确认，提供有容量边界的至少一次投递。
- DeviceSecret直接认证，或ProductSecret动态注册并缓存DeviceSecret；仓库不保存生产密钥。
- 阿里云应用OTA：多文件 `app_fota` HTTP升级，以及兼容旧平台的MQTT分块单 `main.py` 升级。
- 实际预占1.5 MiB OTA空间；体征Flash队列不会使用该空间，并额外保留256 KiB文件系统余量、128 KiB日志额度及运行元数据空间。
- `runtimeLog`、`debugLog`结构化云日志，本地严重日志最多128 KiB。

## 目录

- `src/main.py`：稳定启动和失败回滚入口。
- `src/collector_*.py`：配置、UART、队列、云连接、OTA、日志和传感器模块。
- `src/device.example.json`：设备配置模板。
- `doc/model.zip`：可直接导入阿里云的物模型；`doc/model/`为可编辑源文件。
- `doc/Hardware Protocol-Collector Hardware Send Protocol.png`：Collector硬件通信协议图。
- `fw/QPY_OCPU_EC600M_CNLE_FW_V0006/`：当前EC600M-CN-LE V0606固件及版本说明，根目录同时保留完整压缩包。
- `tools/build_ota_package.py`：生成多文件及旧版单文件OTA制品和MD5清单。
- `tests/`：主机协议、队列、OTA、云确认及物模型测试。

## 首次部署

1. 将 `src/main.py` 和全部 `src/collector_*.py` 下载到设备 `/usr/`。
2. 以 `src/device.example.json` 为模板创建 `/usr/config/device.json`：填写ProductKey、DeviceName、MQTT实例地址，并填写DeviceSecret；若使用动态注册则只填写ProductSecret。
3. 将 `doc/model.zip` 导入对应阿里云产品物模型。
4. 重启设备。设备会先建立 `/usr/.ota_space.reserve`，随后按剩余空间创建体征Flash队列。

`SendCommand`仅为兼容保留。设备收到设置请求会回复成功并记录runtimeLog，但不会向UART发送任何内容。

`runtimeLog`的数值字段使用 `metricValue`。QH-D200模型中的原字段名 `value` 是阿里云物模型保留标识符，不能用于快速导入；构建脚本会在打包前检查全部自定义标识符。

## 构建与测试

```powershell
python -m unittest discover -s tests -v
python tools/build_model_zip.py
python tools/build_ota_package.py
```

物模型脚本会校验 `doc/model/*.json` 并重建 `doc/model.zip`。OTA输出位于 `dist/collector_app_4.0.1/`；构建会按每个文件4 KiB、目录8 KiB计入占用，超过512 KiB时失败。旧版单文件包只更新稳定启动器，使用前必须确认设备上已经部署配套的 `collector_*.py`；从3.x单体版本迁移应先做完整首装或多文件升级。详细上线步骤和实机验收见 `doc/EC600M部署与验收.md`。

## 投递边界

在设备持续供电、UART硬件正常且积压不超过启动日志公布的RAM+Flash容量时，程序不会静默丢包。Flash数据可跨重启恢复；仅存在于RAM的数据无法抵御突然断电。回复丢失会使用相同消息ID重试，因此服务端可能收到重复数据，应按设备和消息ID幂等处理。
