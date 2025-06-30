import app_fota
import fota
import ql_fs
import ujson
from misc import Power
import _thread
import sys_bus

class Lock(object):
    def __init__(self):
        self.lock = _thread.allocate_lock()

    def __enter__(self):
        self.lock.acquire()

    def __exit__(self, *args):
        self.lock.release()

class OTAPlugin(object):
    OTA_FLAG_CONFIG = "/usr/ali_ota_config.json"
    UPGRADE = 1

    from usr.Aliyun_sysbus_Chic_OTA import SYSTOPIC_Class
    SYSTOPIC = SYSTOPIC_Class()
    def __init__(self):
        self.ota_getfirmware_reply = None
        self.ota_getfirmware = None
        self.ota_topic = None
        self.ota_device_process = None
        self.ota_device_inform = None
        self.ota_config = None
        self.id = 0
        self.id_lock = Lock()

    def init(self, cloud):
        self.ota_getfirmware_reply = "/sys/{}/{}/thing/ota/firmware/get_reply".format(
            cloud.productKey, cloud.DeviceName)
        self.ota_getfirmware = "/sys/{}/{}/thing/ota/firmware/get".format(
            cloud.productKey, cloud.DeviceName)
        self.ota_topic = "/ota/device/upgrade/{}/{}".format(
            cloud.productKey, cloud.DeviceName)
        self.ota_device_process = "/ota/device/progress/{}/{}".format(
            cloud.productKey, cloud.DeviceName)
        self.ota_device_inform = "/ota/device/inform/{}/{}".format(
            cloud.productKey, cloud.DeviceName)
        self.ota_config = {
            "version": "V1.0.0",  #脚本升级实验
            # "version": "QPY_OCPU_V0006_EC600N_CNLA_FW", #差分升级实验
            "flag": 0
        }

    def get_id(self):
        with self.id_lock:
            self.id += 1
        return self.id

    def before_start(self, cloud):
        """启动云服务前调用, 可以在云服务启动前配置plugin属性"""
        if not ql_fs.path_exists(self.OTA_FLAG_CONFIG):
            ql_fs.touch(self.OTA_FLAG_CONFIG, self.ota_config)
        else:
            self.ota_config = ql_fs.read_json(self.OTA_FLAG_CONFIG)

    def after_start(self, cloud):
        """云服务启动后调用, 可以获得云服务对象"""
        if self.ota_config.get("flag"):
            self.upload_process(cloud, 100, "升级成功")
            self.ota_config["flag"] = 0
            ql_fs.touch(self.OTA_FLAG_CONFIG, self.ota_config)
        # print("OTAPlugin after_plugin has excute")
        # print("self ota_device_inform = {}".format(self.ota_device_inform))
        self.upload_inform(cloud)

    def get_firmware_reply(self, cloud):
        cloud.ali_subscribe_topic(self.ota_getfirmware_reply)

    def get_firmware(self, cloud):
        version = self.ota_config["version"]
        data = {
                "id": self.get_id(),
                "version": "1.0",
                "params": {
                },
                "method": "thing.ota.firmware.get"
            }
        # cloud.ali_publish(self.ota_getfirmware, ujson.dumps(data))
        msg = {"topic": self.ota_getfirmware, "msg": ujson.dumps(data)}
        sys_bus.publish_sync(self.SYSTOPIC.PUB, msg)

    def process(self, cloud, topic, msg):
        """当收到同类型的plugin的主题时会的到对应的msg和cloud云服务对象"""
        print("process(self, cloud, topic, msg):",msg)
        msg = ujson.loads(msg)
        # if msg.get("code") == "1000":
        if msg.get("code") == 200 or msg.get("code") == "1000":
            data = msg.get("data")
            version = data.get("version")
            if version != None and version != self.ota_config["version"]:
                self.download(cloud, data)
            else:
                print("没有更新版本，无需升级")

    def upload_inform(self, cloud):
        version = self.ota_config["version"]
        data = {
            "id": self.get_id(),
            "params": {
                "version": version,
                "other": self.ota_config
            }
        }
        # cloud.ali_publish(self.ota_device_inform, ujson.dumps(data))
        msg = {"topic": self.ota_device_inform, "msg": ujson.dumps(data)}
        sys_bus.publish_sync(self.SYSTOPIC.PUB, msg)

    def upload_process(self, cloud, step, desc):
        data = {
            "id": self.get_id(),
            "params": {
                "step": str(step),
                "desc": desc
                # "module": "MCU"
            }
        }
        # cloud.ali_publish(self.ota_device_process, ujson.dumps(data))
        msg = {"topic": self.ota_device_process, "msg": ujson.dumps(data)}
        sys_bus.publish_sync(self.SYSTOPIC.PUB, msg)

    def download(self, cloud, data):
        self.upload_process(cloud, 0, "开始下载程序")

        ext_data = data.get("extData")
        if ext_data:
            file_maps = ujson.loads(data["extData"]["_package_udi"])
        else:
            file_maps = dict()
        # 区分对待
        if data.get("isDiff", 0):
            _fota = fota()
            files = data.get("files")
            if files:
                self._firewall_bulk_file_upgarde(
                    cloud, data, _fota, files, file_maps)
            else:
                self._firewall_single_file_upgarde(
                    cloud, data, _fota, files, file_maps)
        else:
            _fota = app_fota.new()
            files = data.get("files")
            if files:
                self._app_bulk_file_upgrade(
                    cloud, data, _fota, files, file_maps)
            else:
                self._app_single_file_upgrade(
                    cloud, data, _fota, files, file_maps)

    def _app_single_file_upgrade(self, cloud, data, _fota, files, file_maps):
        file_name = file_maps.get("upgrade_file", "/usr/main_Chic.py")
        code = _fota.download(data["url"], file_name)
        if code:
            self.upload_process(cloud, -2, "下载文件失败 失败文件如下 {}".format(data))
        else:
            _fota.set_update_flag()
            self.update_config(cloud, data)
            print("升级重启")
            Power.powerRestart()

    def _app_bulk_file_upgrade(self, cloud, data, _fota, files, file_maps):
        file_verify_error = []
        file_verify_success = []
        for file in files:
            file_name = '/usr/'+file["fileName"].replace(".bin", ".py")
            if file_name:
                file_verify_success.append(
                    dict(url=file["fileUrl"], file_name=file_name))
            else:
                file_verify_error.append(
                    dict(url=file["fileUrl"], file_name=file_name))
        if file_verify_error:
            self.upload_process(
                cloud, -2, "文件信息不完整, 无法获取, files_map={} verify_error_files = {}".format(
                    files, file_verify_error))
        else:
            error_download = _fota.bulk_download(file_verify_success)
            if error_download:
                self.upload_process(
                    cloud, -2, "下载文件失败 失败文件如下 {}".format(error_download))
            else:
                _fota.set_update_flag()
                self.update_config(cloud, data)
                print("升级重启")
                Power.powerRestart()

    def _firewall_single_file_upgarde(
            self, cloud, data, _fota, files, file_maps):
        url = data["url"].replace("https", "http")
        self.update_config(cloud, data)
        print("一键升级")
        _fota.httpDownload(url1=url)

    def _firewall_bulk_file_upgarde(
            self, cloud, data, _fota, files, file_maps):
        url1 = ""
        url2 = ""
        for file in files:
            file_name = file["fileName"]
            if file_name == "dfota_1.bin":
                url1 = file["fileUrl"].replace("https", "http")
            if file_name == "dfota_2.bin":
                url2 = file["fileUrl"].replace("https", "http")
        if url1 and url2:
            self.update_config(cloud, data)
            # print('data={}, files={}'.format(data, files))
            # print('url1={}, url2={}'.format(url1, url2))
            print("一键升级")
            _fota.httpDownload(url1=url1, url2=url2)
        else:
            self.upload_process(
                cloud, -2, "下载文件失败 失败文件如下 1阶段地址 {} 2阶段地址{}".format(url1, url2))

    def update_config(self, cloud, data):
        self.upload_process(cloud, 50, "准备重启更新程序")
        self.ota_config["version"] = data.get("version")
        self.ota_config["flag"] = self.UPGRADE
        ql_fs.touch(self.OTA_FLAG_CONFIG, self.ota_config)

