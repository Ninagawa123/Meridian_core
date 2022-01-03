# Meridian_core
  
Meridian計画はヒューマノイド制御についてのオープンソースプロジェクトです。  
汎用で軽量な中間プロトコルを用いることにより、デバイス間で高速に状態データを共有します。  
5~10msの更新頻度でPCのシミュレーション画面とロボット実機をリンクします。  
また、複数社のコマンドサーボ、ROS/Unityなどの多岐にわたるシミュレーション環境、開発環境と自由につなぐことができます。  
  
全体の仕組みや概要は以下のnoteにまとめています。  
https://note.com/ninagawa123/n/ncfde7a6fc835  
  
またプログラムのフローについては「Meridian_system_docs」フォルダの中に資料をまとめています。

Meridian_coreはMeridianの基本となる形で、動作可能なデモを公開しています。  
デモは近藤科学のICSサーボが最低１つあれば試すことができます。  
  
  
  
# System composition
  
Meridianのハードウェアは通信用のESP32DevKitC、制御用のTeensy4.0とそれを連結する専用ボードのMeiridianBoardからなります。  
デモは近藤サーボ(通信速度12.5Mbps）に対応しており、Meiridian Board Type.KはKHR-3HV用に搭載することができます。（専用ボードの回路図は公開しており、自作することも可能です。）  
PC側はROS1のmelodic,noeticに対応しており、現在Rvizでの表示が可能です。またUnity(Mac版）でもヒューマノイドの姿勢をリアルタイム表示することができます。  
  
  
  
# Installation
Teensy4.0、ESP32DevKitCそれぞれのファイルを設定し書き込みます。  
ArduinoIDEやVScode+PlatformIOを使うことができますが、ここではArduinoIDEでの導入について説明します。  
  
## ArduinoIDEのTeensy,ESP32対応
https://qiita.com/Ninagawa_Izumi/items/a8957cc83fe4fbb75759  
別途Qiita記事としてまとめましたので必要な方はご覧ください。  
  
## Teensy4.0の準備  
  
### Arduinoスクリプトの保存場所の確認
ArduinoIDEを起動し、メニューから「ファイル」→「名前をつけて保存」を選び、ファイルが保存される場所を確認します。  
  
### スクリプトのダウンロード
https://github.com/Ninagawa123/Meridian_core  
より、緑色のボタン「code」を押してDownload zipを選びます。  
ファイルを解凍後、以下の２つのファイルを先に確認したArduinoのスクリプトが入っているフォルダにコピーします。  
* 「ESP32」の中の「Meridian_core_for_ESP32_PassThrough」フォルダ  
* 「Teensy」の中の「Meridian_core_for_Teensy40」フォルダ  
  
### Teensy4.0用のファイル設定と書き込み
Teensy4.0本体とPCをUSBで接続します。  
*※信号線を含まない充電専用のUSBコードでは接続できません。  
※Teensy4.0はMeridianBoardに接続したままでも大丈夫ですが、MeridianBoardの供給電源はオフにしてください。Type.Kでは電源コネクタとなりのジャンパピンを抜くことでもTeensy,ESP32への電源供給をオフにできます。*   
メニューから「ファイル」→「開く」を選び、「Meridian_core_for_Teensy4」の中の「Meridian_core_for_Teensy4.ino」を開きます。  
メニューから「ツール」→「ボード」でTeensy4.0を選びます。  
  
#### ライブラリを導入する
メニューから「スケッチ」→「ライブラリのインクルード」→「ライブラリの管理...」を選びライブラリマネージャを開きます。  
検索テキストボックスに「Madgwick」を入力し、 「Madgwick by Arduino」と「Adafruit AHRS by Adafruit」をインストールします。Adafruit AHRSのダイアログボックスではではInstall allを選択します。  
同様に、検索テキストボックスを利用し、  
* 「MPU6050 by Electronic Cats」  
* 「TsyDMASPI by hideakitai」  
* 「ESP32DMASPI by hideakitai」  
をそれぞれインストールします。  
  
* 「IcsHardSerialClass」  
https://kondo-robot.com/faq/ics-library-a2 より「ICS_Library_for_Arduino_V2.1」をDLし解凍後、ArduinoIDEの「スケッチ」→「ライブラリをインクルード」→「.ZIP形式のライブラリをインストール...」を選び、IcsClassV210.zipを選択してインストールします。  
  
#### Teensy4.0にスクリプトを書き込む
スクリプトを書き込みます。  
  
## ESP32の準備
PCにESP32DevKitCをUSBで接続し、  
メニューから「ファイル」→「開く」で「Meridian_core_for_ESP32_PassThrough」の中の「Meridian_core_for_ESP32_PassThrough.ino」を選びます。  
  
### wifiを設定する
91行目-93行目について、接続したいwifiのアクセスポイントのSSIDとパスワードを入力します。  
*アクセスポイントは5GHzではなく**2.4GHz**に対応している必要があります。*  
また、接続先のPCのIPアドレスも記入します。  
  
### 接続先のPCのIPアドレスの調べ方
windowsのコマンドプロンプトを開き、  
$ ipconfig  
と入力しコマンド実行します。  
IPv4アドレスが表示されます（192.168.1.xxなど)  
*Macの場合は画面右上のwifiマークから”ネットワーク”環境設定...で表示されます。*  
  
### ESP32にスクリプトを書き込む
PCとESP32DecKitCをUSBで接続し、  
メニューから「ツール」→「ボード」→「ESP32 Arduino」→「ESP32 Dev Module」を選びます。  
また、「ツール」→「シリアルポート」→で該当のシリアルポートを選びます。  
  
### ESP32のIPアドレスを調べる
ArduinoIDEのシリアルモニタを開き、bpsを2000000に設定します。 ESP32DevKitC本体のENボタンを押します。  
wifi接続に成功すると  
Connecting to WiFi to : (アクセスポイントSSID名) WiFi connected. WiFi connected. ESP32's IP address is : 192.168.x.xx  
と表示され、ESP32本体のIPアドレスが表示されます。この番号をメモしておきます。  
  
## ロボットの姿勢とサーボを設定する
接続するサーボの通信速度設定を**1.25Mbps**に変更します。  
  
また、サーボの0度状態を下記の姿勢に、またサーボの＋回転方向も下図の矢印方向に合わせます。  
*左半身および体の中心は下図に順次つつ、右半身については左半身のミラー方向に回転に合わせます。*  
*サーボの回転方向は、サーボの自身設定以外にも、Teensyスクリプトの261行目以降でも変更できます*  
  
<img width="600" alt="motorccw" src="https://user-images.githubusercontent.com/8329123/147812253-e6cbe388-f70a-445f-80c0-b4cd899aa15a.png">
  
### サーボのマウントを設定
Teensyスクリプトの261行目以降で、サーボのマウントを変更できます。  
接続しているサーボIDは1に、接続していないIDは0に設定します。  
サーボのマウント設定により、KHR-3HVのフルセットがなくてもICSサーボが最低１つあればデモをテストすることができます。  
*サーボ設定に対し１箇所でも接続されていない箇所があると動作しません。存在しないサーボからの返信を待ち続けるためです。*  
  
### サーボの接続
https://github.com/Ninagawa123/Meridian_core/blob/main/Meridian_Board_TypeK_docs/Meridian_pinassign.png  
こちらのピンアサインを参考に、サーボを接続します。  
  
  
  
# ROS版デモを実行する
  
## ROS noeticの導入
お手持ちの環境にROSを導入してください。  
Raspberry py4でROS-noeticを導入する手順については下記にまとめました。  
https://qiita.com/Ninagawa_Izumi/items/e84e9841f7a048832fcc  
  
## URDFの表示テスト
https://github.com/Ninagawa123/roid1_urdf  
まず、こちらのREADMEにしたがってRvizでロボットを表示できるか確認します。  
  
## PCやラズパイ自身のIPアドレスを調べる
ターミナルで「ip a」を入力し、PCやラズパイ等のIPアドレスを調べます。  
wlan0以下の192.168.1.xxの番号を調べてメモします。  
*PCやラズパイはアクセスポイントに無線/有線LANで接続されている必要があります*  
*Macの場合は画面右上のwifiマークから”ネットワーク”環境設定...で表示されます。*  
  
## ESP32の設定を変更する
ESP32の書き込みができるPC等に移動し、  
Meridian_core_for_ESP32_PathThrough.ino を開きます。  
93行目の #define SEND_IP =”192.168.1.xx”の数値を調べた番号に書き換え、 ESP32DevKitCに書き込みます。  
  
## ROSパッケージの作成
PCやラズパイにもどり、ターミナルで作業します。  
  
### ROSパッケージを作成する
$ cd ~/catkin_ws/src  
$ catkin create pkg meridian_demo --catkin-deps roscpp rospy std_msgs  
  
### ROSパッケージにデモのコードを含める
$ cd meridian_demo/src  
$ wget https://raw.githubusercontent.com/Ninagawa123/Meridian_core/main/ROS_demo/rosnode_meridim_demo.py  
  
### 受信ポートと送信ポートの番号をかきかえ、実行可能にする
$ nano rosnode_meridim_demo.py （もしくはcode,vimなどを実行）  
↑エディタでファイルを開き、15行目,18行目のIPアドレスをそれぞれ現在の環境のものに書き換えます。  
$ chmod 755 rosnode_meridim_demo.py  
↑念のためスクリプトを実行可能ファイルにします。 

### パッケージをビルドする
$ cd ~/catkin_ws/  
$ source devel/setup.bash  
$ catkin build  
$ source ~/catkin_ws/devel/setup.bash  
  
## ROS,rviz,meridian_demo実行する
１つ目のターミナルを開き、  
$ roscore  
  
２つ目のターミナルを開き、  
$ roslaunch roid1_urdf display_meridian_demo.launch  
*この時点ではロボットはベースとなる腰部分しか表示されません*  
  
３つ目のターミナルを開き、  
$ rosrun meridian_demo rosnode_meridim_demo.py  
  
MeridianBoardの電源を入れると、ロボットのサーボ位置が画面の表示に反映されます。  
  
  
  
#  Unity版デモを実行する（※Macのみ動作します。Winでは現状エラーが発生します。）
  
###  UnityHubに登録して起動する
フォルダ「Unity_demo」の中のMeridian_unity_demo_mac_20211231.zipを解凍します。  
UnityHubを開き、ProjectsのADDで解凍済みの「Meridian_unity_demo_mac_20211231」フォルダを指定します。  
UnityHubに登録されたらプロジェクトを起動します。（Unityのバージョンは2020.3.25f1(LTS)です。） 
  
###  UnityのスクリプトのIPアドレスを書き換える
画面下の「Project」→「Assets」→「Script」よりUdp_handler_sendをダブルクリックして開きます（VScodeなどが立ち上がります）  
スクリプト9行目のconst string HOST = "192.168.1.xx"; にESP32DevKitCのIPアドレスを記入し、セーブします。
  
###  ESP32のIPアドレスを書き換える
これまでの手順で設定済みの場合はそのままでOKです。
Meridian_core_for_ESP32_PathThrough.inoの93行目にUnityを使うPCのIPアドレスを入力します。

###  UnityとMeridianボードを起動する
Unityを起動した後、Meridianボードを
通信が成立していればロボットの関節にシンクロして画面の中のモデルが動きます。

画面の「Send」にチェックを入れるとUnityからロボットを操作することができます。
スライドバーに対応したサーボが動きます。またスライドバーの上のテキストボックスに直接数値を入力することができます。
数値の単位はdegreeとなります。

「Action」にチェックを入れるとUnityからロボットにサンプルのモーションを送信します。
左半身の各関節角度をsinカーブで増減させます。
このモーションはParamMaster.csの160行目-167行目で設定しているので、適宜書き換えてお試しください。

