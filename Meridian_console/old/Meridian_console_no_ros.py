# #!/usr/bin/python3
# coding: UTF-8
# もしくは　#!/usr/bin/env python など環境に合わせて

#2022.02.05 UDP通信動作は安定。
#2022.02.05 COMMAND WINDOWのPOWERチェックボックスでサーボ電源ON
#2022.02.05 上記サーボ電源ON中にスライドバー操作でサーボ動作（ただしスライドバーが小さいため大まかな動作確認のみに利用可）


import numpy as np
import socket
from contextlib import closing
import struct
import math
import dearpygui.dearpygui as dpg
import threading
import signal
import time
import random

UDP_RESV_IP="192.168.1.xx" #このPCのIPアドレス
UDP_RESV_PORT=22222 #受信ポート

UDP_SEND_IP="192.168.1.xx" #送信先のESP32のIPアドレス
UDP_SEND_PORT=22224 #送信ポート

MSG_SIZE = 90
MSG_BUFF = MSG_SIZE * 2

UPDATE_YAW_CENTER_FLAG = 0
UPDATE_YAW_CENTER_NUM = 1002
SERVO_POWER_FLAG = 0

sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)#UDP用のsocket設定
sock.bind((UDP_RESV_IP,UDP_RESV_PORT))

data2 = []
loop_count = 0
error_count_pc_udp = 0
error_count_esp_udp = 0
error_count_tsy_spi = 0
flag_stop = 0
start = time.time()

r_meridim_disp=list(range(MSG_SIZE))
r_meridim_disp_char=list(range(MSG_SIZE*2))
s_meridim=[0]*MSG_SIZE
s_meridim_motion=[0]*MSG_SIZE #PC側で作成したサーボ位置命令
message0 = "This PC's IP adress is "+UDP_RESV_IP
message1 = ""
message2 = ""
button = "button"
test = 1
r_udp_flag = 0


def udpresv():
    pass

#データの送受信
def main():
    global message0
    global message1
    global message2

    while (True):
        message1 = "Waiting for UDP data from "+UDP_SEND_IP+"..."
        with closing(sock):
            while True:

                global loop_count
                global r_meridim_disp
                global r_meridim_disp_char
                global s_meridim_motion
                global test
                global error_count_esp_udp
                global error_count_tsy_spi
                global SERVO_POWER_FLAG

                loop_count += 1
                r_bin_data,addr = sock.recvfrom(1472)
                r_meridim=struct.unpack('90h',r_bin_data)
                r_meridim_disp_char=struct.unpack('180b',r_bin_data)
                message1 = "UDP data receiving from "+UDP_SEND_IP

                checksum = np.array([0], dtype=np.int16)
                for i in  range(MSG_SIZE-1):
                    checksum[0] += r_meridim[i]
                    r_meridim_disp[i]=r_meridim[i]
                checksum[0] = ~checksum[0]

                temp = np.array([0], dtype=np.int16)

                if checksum[0] == r_meridim[MSG_SIZE-1]:
                    #print(r_meridim_disp[88])
                    if (r_meridim_disp[88] >> 14 & 1) == 1:#エラーフラグ14ビット目（ESP32UDP受信エラーフラグ）を調べる
                        error_count_esp_udp += 1
                    if (r_meridim_disp[88] >> 13 & 1) == 1:#エラーフラグ14ビット目（ESP32UDP受信エラーフラグ）を調べる
                        error_count_tsy_spi += 1
                    temp[0] = r_meridim[88] & 32767 #配列のエラーフラグ15番(PCのUDP受信エラー判定)を下げる

                else:
                    temp[0] = r_meridim[88] | 32768 #配列のエラーフラグ15番(PCのUDP受信エラー判定)を上げる                    
                    global error_count_pc_udp        
                    error_count_pc_udp += 1
                signal.signal(signal.SIGINT, signal.SIG_DFL)

                #time.sleep(1)

                #PC側サーボ位置発信用に最終サーボ情報をキープ
                if SERVO_POWER_FLAG == 2: #サーボオンボタン押下初回のみ最終受け取りサーボ情報をキープ
                    for i in  range(21,81,2):
                        s_meridim_motion[i] = r_meridim[i]
                    SERVO_POWER_FLAG = 1

                #送信データを作成する
                s_meridim=[]
                s_meridim=list(r_meridim)

                #キープしたエラーフラグを格納
                s_meridim[88] = temp[0]


                #サーボオンオフフラグを格納
                if SERVO_POWER_FLAG > 0:
                    for i in  range(20,80,2):
                        s_meridim[i] = 1
                        s_meridim[i+1] = s_meridim_motion[i+1]
                else:
                    for i in  range(20,80,2):
                        s_meridim[i] = 0

                #ヨー軸センターリセットコマンド
                global UPDATE_YAW_CENTER_FLAG
                if (UPDATE_YAW_CENTER_FLAG > 0):
                    UPDATE_YAW_CENTER_FLAG -= 1 
                    s_meridim[0] = UPDATE_YAW_CENTER_NUM
                    if (UPDATE_YAW_CENTER_FLAG==0):
                        print("Set Yaw Center. COMMAND: "+str(UPDATE_YAW_CENTER_NUM))

                #格納した送信データについてチェックサムを追加
                checksum[0] = 0
                checksum_int = 0
                for i in  range(MSG_SIZE-1):
                    checksum_int += s_meridim[i]
                checksum[0] = ~checksum_int
                s_meridim[MSG_SIZE-1]=checksum[0]

                #データをパックしてUDP送信
                s_bin_data=struct.pack('90h',*s_meridim)
                sock.sendto(s_bin_data,(UDP_SEND_IP,UDP_SEND_PORT))

                now = time.time()-start
                message2="ERROR PC-UDP:"+str("{:.2%}".format(error_count_pc_udp/loop_count))+\
                    "  ESP-UDP:"+str("{:.2%}".format(error_count_esp_udp/loop_count))+"  TSY-SPI:"+str("{:.2%}".format(error_count_tsy_spi/loop_count))+\
                    "  Frames:"+str(loop_count)+"  "+str(int(loop_count/now))+"Hz"  





def set_servo_power():#チェックボックスに従いパワーをオンオフ
    global SERVO_POWER_FLAG
    if SERVO_POWER_FLAG == 0 :
        SERVO_POWER_FLAG = 2
        print("Servo Power ON")
    else:
        SERVO_POWER_FLAG = 0
        print("Servo Power OFF")

def set_servo_angle(sender, app_data):#チェックボックスに従いパワーをオンオフ
    global s_meridim_motion
    if sender[3]=="L":
        s_meridim_motion[int(sender[4:6])*2+21] = int(app_data*100)
        print(f"L meri: {int(sender[4:6])*2+21}")
    if sender[3]=="R":
        s_meridim_motion[int(sender[4:6])*2+51] = int(app_data*100)
        print(f"R meri: {int(sender[4:6])*2+51}")

    print(f"sender is: {sender[3]}")
    print(f"sender is: {sender[4:6]}")
    print(f"app_data is: {int(app_data*100)}")
    print(f"motion is: {s_meridim_motion[int(sender[4:6])+21]}")


#dearpyguiによるコンソール画面描写
def dpgrun():

    def set_yaw_center():#ターミナルにClicked!と表示する
        global UPDATE_YAW_CENTER_FLAG
        UPDATE_YAW_CENTER_FLAG = 10

    dpg.create_context()
    dpg.create_viewport(title='Meridian Console without ROS', width=600, height=480)
    with dpg.window(label="Axis Monitor", width=250, height=350,pos=[5,5]):
        with dpg.group(label='LeftSide'): 
            for i in range(0, 15, 1):
                dpg.add_slider_float(default_value=0, tag="ID L"+str(i),label="L"+str(i),max_value=100,min_value=-100,callback=set_servo_angle,pos=[10,35+i*20], width=80)
        with dpg.group(label='RightSide'):
            for i in range(0, 15, 1):
                dpg.add_slider_float(default_value=0, tag="ID R"+str(i),label="R"+str(i),max_value=100,min_value=-100,callback=set_servo_angle,pos=[135,35+i*20], width=80)

    with dpg.window(label="Messege", width=590, height=115,pos=[5,360]):
        dpg.add_text(message0,tag="DispMessage0")
        dpg.add_text(message1,tag="DispMessage1")
        dpg.add_text(message2,tag="DispMessage2")

    with dpg.window(label="Sensor Monitor", width=335, height=175,pos=[260,5]):
        with dpg.group(label='LeftSide'): 
            dpg.add_slider_float(default_value=0, tag="mpu0", label="ac_x",max_value=327,min_value=-327,pos=[10,35], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu1", label="ac_y",max_value=327,min_value=-327,pos=[115,35], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu2", label="ac_z",max_value=327,min_value=-327,pos=[220,35], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu3", label="gr_x",max_value=327,min_value=-327,pos=[10,55], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu4", label="gr_y",max_value=327,min_value=-327,pos=[115,55], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu5", label="gr_z",max_value=327,min_value=-327,pos=[220,55], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu6", label="mg_x",max_value=327,min_value=-327,pos=[10,75], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu7", label="mg_y",max_value=327,min_value=-327,pos=[115,75], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu8", label="mg_z",max_value=327,min_value=-327,pos=[220,75], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu9", label="temp",max_value=327,min_value=-327,pos=[10,95], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu10", label="rol",max_value=327,min_value=-327,pos=[10,120], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu11", label="pit",max_value=327,min_value=-327,pos=[115,120], width=60)
            dpg.add_slider_float(default_value=0, tag="mpu12", label="yaw",max_value=327,min_value=-327,pos=[220,120], width=60)
            dpg.add_button(label="SetYaw",  callback=set_yaw_center, width =50, pos=[270,148])

    with dpg.value_registry():
        dpg.add_int_value(tag="button_data")

    with dpg.window(label="Command", width=335, height=170,pos=[260,185]):
        dpg.add_checkbox(label="Power", tag="Power",  callback=set_servo_power)#, width =50, pos=[10,30])
        dpg.add_text("Control Pad Monitor", pos=[10,100])
        dpg.add_text("button",tag="pad_button", pos=[170,100])
        dpg.add_slider_int(default_value=0, tag="pad_Lx", label="Lx",max_value=127,min_value=-127, pos=[10,120], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_Ly", label="Ly",max_value=127,min_value=-127, pos=[90,120], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_Rx", label="Rx",max_value=127,min_value=-127, pos=[170,120], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_Ry", label="Ry",max_value=127,min_value=-127, pos=[250,120], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_L2v", label="L2v",max_value=255,min_value=0, pos=[90,140], width=40)
        dpg.add_slider_int(default_value=0, tag="pad_R2v", label="R2v",max_value=255,min_value=0, pos=[170,140], width=40)

    dpg.setup_dearpygui()
    dpg.show_viewport()

    while dpg.is_dearpygui_running():
        for i in range(0, 15, 1):
            global button
            idld = r_meridim_disp[21+i*2]
            idrd = r_meridim_disp[51+i*2]
            idsensor = r_meridim_disp[i+2]/10000
            dpg.set_value("ID L"+str(i), idld/100)
            dpg.set_value("ID R"+str(i), idrd/100)
            dpg.set_value("DispMessage0", message0)
            dpg.set_value("DispMessage1", message1)
            dpg.set_value("DispMessage2", message2)
            if i < 13:
                if i < 11:
                    dpg.set_value("mpu"+str(i),idsensor)
                else:
                    dpg.set_value("mpu"+str(i),idsensor*100)

        dpg.set_value("pad_button", str(r_meridim_disp[80]))
        dpg.set_value("pad_Lx", r_meridim_disp_char[163])
        dpg.set_value("pad_Ly", r_meridim_disp_char[162])
        dpg.set_value("pad_Rx", r_meridim_disp_char[165])
        dpg.set_value("pad_Ry", r_meridim_disp_char[164])
        padL2val = (r_meridim_disp_char[167])
        if (padL2val<0):
            padL2val = 256+padL2val
        if (r_meridim_disp[80]&256==0):
            padL2val = 0
        padR2val = (r_meridim_disp_char[166])
        if (padR2val<0):
            padR2val = 256+padR2val
        if (r_meridim_disp[80]&512==0):
            padR2val = 0
        dpg.set_value("pad_L2v", padL2val)
        dpg.set_value("pad_R2v", padR2val)
        dpg.set_value("button_data", r_meridim_disp[80])

        dpg.render_dearpygui_frame()

    dpg.destroy_context()

#スレッド2つで送受信と画面描写を並列処理
if __name__ == '__main__':
    thread1 = threading.Thread(target=dpgrun)
    thread1.start()
    thread2 = threading.Thread(target=udpresv)
    thread2.start()
    main()
