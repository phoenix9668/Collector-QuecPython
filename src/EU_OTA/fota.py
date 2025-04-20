#远程升级

import fota
import utime
import log

# 设置日志输出级别
log.basicConfig(level=log.INFO)
fota_log = log.getLogger("Fota")

def result(args):
    print('download status:',args[0],'download process:',args[1])

def run():
    fota_obj = fota()  # 创建Fota对象
    fota_log.info("httpDownload...")
    #差分升级方式
    #res = fota_obj.httpDownload(url1="http://97nm11vh0943.vicp.fun/output.pack",callback=result)    
    #mini fota方式
    res = fota_obj.httpDownload(url1="http://139.155.96.99:7000/test2/dfota_1.bin",url2="http://139.155.96.99:7000/test2/dfota_2.bin")
    #差分升级FTP方式  
    #res = fota_obj.ftpDownload(url1="ftp://test:test@220.180.239.212:8309/dfota_11.bin",url2="ftp://test:test@220.180.239.212:8309/dfota_12.bin",callback=result)  #其中user,password,ip,port需要填写具体使用的FTP服务器信息
    if res != 0:
        print("httpDownload error  ",res)
        return
    fota_log.info("wait httpDownload update...")
    utime.sleep(2)

if __name__ == '__main__':
    fota_log.info("run start...")
    run()    
