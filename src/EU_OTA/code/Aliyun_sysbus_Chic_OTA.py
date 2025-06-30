
import utime
import checkNet
from aLiYun import aLiYun
import _thread
import sys_bus


class SYSTOPIC_Class(object):
    RRPC = "rrpc"
    OTA = "ota"
    PUB = "pub"
    SUB = "sub"


class aliyun_Class(object):
    def __init__(self):
        aliyun_Class.inst = self
        self.productKey = ""  # 产品标识
        self.productSecret = None  # 'mj7qKfEn73y07gyK'  # 产品密钥（一机一密认证此参数传入None）
        self.DeviceSecret = "" # None  # 设备密钥（一型一密认证此参数传入None）
        self.DeviceName = ""  # 设备名称
        # /broadcast/a1llZotKkCm/123
        self.subscribe_topic1 = "/broadcast" + "/" + self.productKey + "/" + "123"
        self.subscribe_topic2 = "/broadcast" + "/" + self.productKey + "/" + "123"
        self.publish_topic1 = "/broadcast" + "/" + self.productKey + "/" + "123"

        # 创建aliyun连接对象
        # MqttServer - 可选参数,需要连接的服务器名称,默认为"{productKey}.iot-as-mqtt.cn-shanghai.aliyuncs.com"，字符串类型
        self.ali = aLiYun(
            self.productKey,
            self.productSecret,
            self.DeviceName,
            self.DeviceSecret)
        # 设置mqtt连接属性
        clientID = b'clientID'  # 自定义字符（不超过64）
        ret = self.ali.setMqtt(
            clientID,
            clean_session=False,
            keepAlive=60,
            reconn=True)  # False True

        # 设置回调函数
        self.ali.setCallback(self.ali_sub_cb)

    def ali_sub_cb(self, topic, msg):  # 回调函数
        if topic.decode().find(SYSTOPIC.RRPC) != -1:
            sys_bus.publish(SYSTOPIC.RRPC, {"topic": topic, "msg": msg})
        elif topic.decode().find(SYSTOPIC.OTA) != -1:
            sys_bus.publish(SYSTOPIC.OTA, {"topic": topic, "msg": msg})
        else:
            sys_bus.publish(SYSTOPIC.SUB, {"topic": topic, "msg": msg})

    def ali_start(self):
        # 运行
        self.ali.start()
        print('Runing')
        # aLiYun.disconnect()

    def ali_subscribe_topic(self, _topic):
        # 订阅主题
        self.ali.subscribe(_topic, qos=0)

    def ali_publish(self, topic, msg):
        ret = self.ali.getAliyunSta()
        # print(ret)
        if ret == 0:
            try:
                self.ali.publish(msg.get('topic'), msg.get("msg"), qos=0)
            except BaseException:
                print('！！！！！！！！！！发送失败')


class Handler(object):
    @classmethod
    def sub(cls, topic, msg):
        print(
            "Subscribe Recv: Topic={},Msg={}".format(
                msg.get('topic').decode(),
                msg.get("msg").decode()))


    @classmethod
    def pub(cls, msg):
        while True:
            sys_bus.publish(SYSTOPIC.PUB, msg)
            utime.sleep_ms(10000)

    @classmethod
    def ota(cls, topic, msg):
        """处理完ota的信息后，同步发送"""
        # msg = {"topic": "xxx", "msg": "xxx"}
        # """同步publish，同步情况下会等待所有topic对应的处理函数处理完才会退出"""
        # sys_bus.publish_sync(SYSTOPIC.PUB, msg)

        # print("classmethod  ota()", topic, msg)
        de_topic = msg.get('topic').decode()
        de_msg = msg.get("msg").decode()
        _thread.start_new_thread(AliyunOTA.process, (aliyunClass.ali ,de_topic, de_msg))

    @classmethod
    def rrpc(cls, topic, msg):
        """发布rrpc执行下列操作"""
        msg = {"topic": "xxx", "msg": "xxx"}
        """异步publish， """
        sys_bus.publish(SYSTOPIC.PUB, msg)


if __name__ == '__main__':
    stagecode, subcode = checkNet.wait_network_connected(30)
    print('stagecode = {}, subcode = {}'.format(stagecode, subcode))

    aliyunClass = aliyun_Class()
    aliyunClass.ali_subscribe_topic(aliyunClass.subscribe_topic1)
    # aliyunClass.ali_subscribe_topic(aliyunClass.subscribe_topic2)

    SYSTOPIC = SYSTOPIC_Class()
    sys_bus.subscribe(SYSTOPIC.RRPC, Handler.rrpc)
    sys_bus.subscribe(SYSTOPIC.OTA, Handler.ota)
    sys_bus.subscribe(SYSTOPIC.SUB, Handler.sub)
    sys_bus.subscribe(SYSTOPIC.PUB, aliyunClass.ali_publish)

    tuple = ({"topic": aliyunClass.publish_topic1, "msg": "hello world"},)
    _thread.start_new_thread(Handler.pub, tuple)

    aliyunClass.ali_start()

    from usr.Aliyun_OTA import OTAPlugin
    AliyunOTA = OTAPlugin()
    AliyunOTA.init(aliyunClass)
    AliyunOTA.before_start(aliyunClass)
    AliyunOTA.after_start(aliyunClass)
    AliyunOTA.get_firmware_reply(aliyunClass)
    AliyunOTA.get_firmware(aliyunClass)

