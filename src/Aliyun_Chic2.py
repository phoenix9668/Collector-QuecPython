import utime
import checkNet
from aLiYun import aLiYun
import _thread
import sys_bus
import ujson


class SYSTOPIC_Class(object):
    RRPC = "rrpc"
    OTA = "ota"
    PUB = "pub"
    SUB = "sub"


class aliyun_Class(object):
    def __init__(self):
        aliyun_Class.inst = self
        self.productKey = "a1DTBc9HeFm"  # 产品标识
        self.productSecret = None  # 产品密钥（一机一密认证此参数传入None）
        self.DeviceSecret = "d4c1429d71049a57aa0ec7404e91fffe"  # 设备密钥（一型一密认证此参数传入None）
        self.DeviceName = "quecPythonTestDevice"  # 设备名称
        # /broadcast/a1llZotKkCm/123
        self.subscribe_topic1 = "/sys/a1DTBc9HeFm/quecPythonTestDevice/thing/service/property/set"
        self.publish_topic1 = "/sys/a1DTBc9HeFm/quecPythonTestDevice/thing/event/property/post"

        # 创建aliyun连接对象
        self.ali = aLiYun(
            self.productKey,
            self.productSecret,
            self.DeviceName,
            self.DeviceSecret)
        # 设置mqtt连接属性
        clientID = b'a1DTBc9HeFm.quecPythonTestDevice'  # 自定义字符（不超过64）
        ret = self.ali.setMqtt(
            clientID,
            clean_session=False,
            keepAlive=60,
            reconn=True)  # False True
        # print("setMqtt = {}".format(ret))
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

    def ali_subscribe_topic(self):
        # 订阅主题
        self.ali.subscribe(self.subscribe_topic1, qos=0)
        # self.ali.subscribe(self.subscribe_topic2, qos=0)

    def ali_publish(self, topic, msg):
        ret = self.ali.getAliyunSta()
        # print("getAliyunSta = {}".format(ret))
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

        ret = msg.get("msg").decode()
        ret = ujson.loads(ret)
        print(type(ret))
        print(ret["method"])
        print(ret["id"])
        print(ret["params"])
        print(ret["params"]["MotionAlarmState"])
        print(ret["params"]["CurrentTemperature"])

    @classmethod
    def pub(cls, msg):
        while True:
            sys_bus.publish(SYSTOPIC.PUB, msg)
            utime.sleep_ms(2000)

    @classmethod
    def ota(cls, topic, msg):
        """处理完ota的信息后，同步发送"""
        msg = {"topic": "xxx", "msg": "xxx"}
        """同步publish，同步情况下会等待所有topic对应的处理函数处理完才会退出"""
        sys_bus.publish_sync(SYSTOPIC.PUB, msg)

    @classmethod
    def rrpc(cls, topic, msg):
        """发布rrpc执行下列操作"""
        msg = {"topic": "xxx", "msg": "xxx"}
        """异步publish， """
        sys_bus.publish(SYSTOPIC.PUB, msg)


if __name__ == '__main__':
    PROJECT_NAME = "QuecPython"
    PROJECT_VERSION = "1.0.0"
    checknet = checkNet.CheckNetwork(PROJECT_NAME, PROJECT_VERSION)
    checknet.poweron_print_once()
    checknet.wait_network_connected()

    aliyunClass = aliyun_Class()
    aliyunClass.ali_subscribe_topic()

    SYSTOPIC = SYSTOPIC_Class()
    sys_bus.subscribe(SYSTOPIC.RRPC, Handler.rrpc)
    sys_bus.subscribe(SYSTOPIC.OTA, Handler.ota)
    sys_bus.subscribe(SYSTOPIC.SUB, Handler.sub)
    sys_bus.subscribe(SYSTOPIC.PUB, aliyunClass.ali_publish)

    msg_topic1 = """{{
              "id": "123",
              "version": "1.0",
              "sys":{{
                  "ack":0
              }},
              "params": {{
                  "mhumi": {{
                  "value": {}
                  }},
                  "mtemp": {{
                  "value": {}
                  }}
              }},
              "method": "thing.event.property.post"
        }}
        """
    a = 0
    b = 0
    for i in range(200):
        a = i
        b = i + 100
        tuple = ({"topic": aliyunClass.publish_topic1, "msg": msg_topic1.format(a, b)},)
    _thread.start_new_thread(Handler.pub, tuple)

    aliyunClass.ali_start()
