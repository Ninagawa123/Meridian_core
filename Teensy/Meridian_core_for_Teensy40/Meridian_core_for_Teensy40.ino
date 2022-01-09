//Meridian_core_for_Teensy_2021.12.31
//This code is for Teensy 4.0

/*
  -------------------------------------------------------------------------
  ---- Teensy4.0 ピンアサイン [S-1-0] --------------------------------------
  -------------------------------------------------------------------------
  [GND]               -> GND
  [00] RX1, CRX2      -> ICS_3rd_TX
  [01] TX1, CTX2      -> ICS_3rd_RX
  [02]                -> LED（lights up when the processing time is not within the specified time.）
  [03]                -> (NeoPixel Data)
  [04]                -> (NeoPixel Clock)
  [05]                -> ICS_Right_EN
  [06]                -> ICS_Left_EN
  [07] RX2            -> ICS_Left_TX
  [08] TX2            -> ICS_Left_RX
  [09]                -> SD_DAT3/CD (SD[2])
  [10] CS             -> SPI_CS (ESP32[15])
  [11] MOSI           -> SPI/SD_MOSI (ESP32[13]) & SD_CMD (SD[3])
  [12] MISO           -> SPI/SD_MISO (ESP32[12]) & SD_DAT0 (SD[7])
  [Vin]               -> 5V
  [AGND]              ->
  [3.3v]              -> GY-521(MPU6050) 3.3Vin & SD_VDD (SD[4])
  [23] CRX1           -> ICS_3rd_EN
  [22] CTX1           ->
  [21] RX5            ->
  [20] TX5            ->
  [19] I2C-SCL0       -> GY-521(MPU6050) SCL
  [18] I2C-SDA0       -> GY-521(MPU6050) SDA
  [17] TX4, I2C-SDA1  -> (PC/Raspi etc.)
  [16] RX4, I2C-SCL1  -> (PC/Raspi etc.)
  [15] RX3            -> ICS_Right_TX
  [14] TX3            -> ICS_Right_RX
  [13] SCK(CRX1)      -> SPI/SD_SCK (ESP32[14]) & SD_CLK (SD[5])
  -------------------------------------------------------------------------
  ---- サーボIDとロボット部位、軸との対応表 [S-1-1] ----------------------------
  -------------------------------------------------------------------------
  ＜ICS_Left_Upper SIO1,SIO2＞
  ID    Parts-Axis
  [L01] 左肩ピッチ
  [L02] 左肩ロール
  [L03] 左肘ヨー
  [L04] 左肘ピッチ
  [L05] ー
  ＜ICS_Left_Lower SIO3,SIO4＞
  ID    Parts-Axis
  [L00] 腰ヨー
  [L06] 左股ロール
  [L07] 左股ピッチ
  [L08] 左膝ピッチ
  [L09] 左足首ピッチ
  [L10] 左足首ロール
  ＜ICS_Right_Upper SIO5,SIO6＞
  ID    Parts-Axis
  [R00] 頭ヨー
  [R01] 右肩ピッチ
  [R02] 右肩ロール
  [R03] 右肘ヨー
  [R04] 右肘ピッチ
  [R05] ー
  ＜ICS_Right_Lower SIO7,SIO8＞
  ID    Parts-Axis
  [R06] 右股ロール
  [R07] 右股ピッチ
  [R08] 右膝ピッチ
  [R09] 右足首ピッチ
  [R10] 右足首ロール
  -------------------------------------------------------------------------
  ---- Meridim配列 一覧表 [S-1-2] ------------------------------------------
  -------------------------------------------------------------------------
  [00]      マスターコマンド デフォルトは90 で配列数も同時に示す
  [01]      移動時間
  [02]-[04] IMU:acc＿x,acc＿y,acc＿z    加速度x,y,z
  [05]-[07] IMU:gyro＿x,gyro＿y,gyro＿z ジャイロx,y,z
  [08]-[10] IMU:mag＿x,mag＿y,mag＿z    磁気コンパスx,y,z
  [11]      IMU:temp                   温度
  [12]-[14] IMU:DMP ROLL,PITCH,YAW     DMP推定値 ロール,ピッチ,ヨー
  [15]      free
  [16]      free
  [17]      free
  [18]      free
  [19]      free
  [20]      サーボID LO  コマンド
  [21]      サーボID LO  データ値
  ...
  [48]      サーボID L14 コマンド
  [49]      サーボID L14 データ値
  [50]      サーボID RO  コマンド
  [51]      サーボID RO  データ値
  ...
  [78]      サーボID R14 コマンド
  [79]      サーボID R14 データ値
  [80]      free ボタンデータ1
  [81]      free ボタンデータ2
  [82]      free ボタンアナログ1
  [83]      free ボタンアナログ2
  [84]      free (ボタンアナログ3)
  [85]      free (ボタンアナログ4)
  [86]      free (Teensy SPI receive Error Rate)
  [87]      free (ESP32 SPI receive Error Rate)
  [88]      free (PC UDP receive Error Rate)
  [89]      チェックサム
*/

//[S-2] ライブラリ導入 -----------------------------------
#include <Wire.h> //MPU-6050のライブラリ導入
#include <SPI.h> //SDカード用のライブラリ導入
#include <SD.h> //SDカード用のライブラリ導入
#include <TsyDMASPI.h> //SPI Master用のライブラリを導入
#include <MadgwickAHRS.h> //MPU6050のライブラリ導入
#include "MPU6050_6Axis_MotionApps20.h" //MPU6050のライブラリ導入2
#include <IcsHardSerialClass.h> //ICSサーボのライブラリ導入

//[S-3] 各種設定 #DEFINE ---------------------------------
//マウント有無とピンアサイン (S-3-1) ---------------------------------
#define ESP32_MOUNT 1 //0:なし(SPI通信およびUDP通信を実施しない)、1:あり
#define SD_MOUNT 1 //SDカードリーダーのありなし。MeridianBoard Type.Kは有り
#define CHIPSELECT_SD 9 //SDカードSPI通信用のChipSelectのピン番号
#define IMU_MOUNT 0 //IMUの搭載状況 0=off, 1=MPU6050, ...
#define IMU_FREQ 10 //IMUのセンサの読み取り間隔(ms)
#define JOYPAD_MOUNT 0 //ジョイパッドの搭載 0:なし、Wiimote:1, Wiimote+:2, KRC-5FH:3 (※KRC-5FH:3のみ実装済,MeridianBoardではICS_R系に接続)
#define JOYPAD_FRAME 4 //上記JOYPADのデータを読みに行くフレーム間隔 (※KRC-5FHでは4推奨)
#define ICS3_MOUNT 0 //半二重サーボ信号の3系のありなし

//その他の基本設定 (S-3-2) ---------------------------------
#define MSG_SIZE 90 //Meridim配列の長さ設定（デフォルトは90）
#define ERR_LED 2 //LED用 処理が時間内に収まっていない場合に点灯
#define SERIAL_PC 60000000 //PCとのシリアル速度（モニタリング表示用）
#define EN_L_PIN 6 //ICSサーボ信号の左系のENピン番号（固定）
#define EN_R_PIN 5 //ICSサーボ信号の右系のENピン番号（固定）
#define EN_3_PIN 23 //半二重サーボ信号の3系のENピン番号（固定）
#define BAUDRATE 1250000 //ICSサーボの通信速度1.25M
#define TIMEOUT 1000 //ICS返信待ちのタイムアウト時間。通信できてないか確認する場合には1000ぐらいに設定するとよい。

//マスターコマンド定義 (S-3-3) ---------------------------------
#define UPDATE_YAW_CENTER 102 //センサの推定ヨー軸を現在値センターとしてリセット
#define ENTER_TRIM_MODE 103   //トリムモードに入る（全サーボオンで垂直に気おつけ姿勢で立つ）

//タイマー管理用の変数
long frame_ms = 5;// 1フレームあたりの単位時間(ms)
long merc = (long)millis(); // フレーム管理時計の時刻 Meridian Clock.
long curr = (long)millis(); // 現在時刻を取得
long curr_micro = (long)micros(); // 現在時刻を取得
int framecount = 0;//サイン計算用の変数
int framecount_diff = 2;//サインカーブ動作などのフレームカウントをいくつずつ進めるか
int joypad_framecount = 0;//JOYPADのデータを読みに行くためのフレームカウント

//変数一般
static const int MSG_BUFF = MSG_SIZE * 2; //Meridim配列の長さ（byte換算）
int checksum; //チェックサム計算用
int spi_ok = 0; //通信のエラーカウント
int spi_trial = 0; //通信のエラーカウント
bool file_open = 0; //SDカード用の変数
int k; //各サーボの計算用変数
unsigned short button_1 = 0;//受信ボタンデータ1群
unsigned short button_2 = 0;//受信ボタンデータ2群
short stick_Lx = 0;//受信ジョイスティックデータLx
short stick_Ly = 0;//受信ジョイスティックデータLy
short stick_Rx = 0;//受信ジョイスティックデータRx
short stick_Ry = 0;//受信ジョイスティックデータRy

int test_val_1 = 0;//テスト用

//共用体の宣言 : Meridim配列格納用、SPI送受信バッファ配列格納用
typedef union //共用体は共通のメモリ領域に異なる型で数値を読み書きできる
{
  short sval[MSG_SIZE];// short型で100個の配列データを持つ
  uint8_t bval[MSG_BUFF]; //1バイト単位で200個の配列データを持つ
} UnionData;

UnionData s_merdim; //Meridim配列データ(short型、センサや角度は100倍値)
UnionData r_merdim; //Meridim配列データ(short型、センサや角度は100倍値)
UnionData s_packet; //SPI送信用の共用体のインスタンスを作成
UnionData r_packet; //SPI受信用の共用体のインスタンスを作成

//MPU6050のアドレス、レジスタ設定値
MPU6050 mpu;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector
float ROLL, PITCH, YAW, YAW_ZERO;

//ICSサーボのインスタンス設定
IcsHardSerialClass krs_L(&Serial2, EN_L_PIN, BAUDRATE, TIMEOUT);
IcsHardSerialClass krs_R(&Serial3, EN_R_PIN, BAUDRATE, TIMEOUT);
IcsHardSerialClass krs_3(&Serial1, EN_3_PIN, BAUDRATE, TIMEOUT);//3系もICSの場合

//KRSサーボのポジション用配列(degreeではなくサーボ値が入る)
int s_KRS_servo_pos_L[] = {7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500}; //15要素
int s_KRS_servo_pos_R[] = {7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500}; //15要素
int r_KRS_servo_pos_L[] = {7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500}; //15要素
int r_KRS_servo_pos_R[] = {7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500, 7500}; //15要素

//各サーボの各サーボのマウント判定用配列
bool idl_mt[15];//idlのありなし（mtはmountの略）
bool idr_mt[15];//idrのありなし（mtはmountの略）

//各サーボの内外回転プラマイ方向補正用配列
float idl_pn[15];//idlのプラマイ（pnはposi/negaの略）
float idr_pn[15];//idrのプラマイ（pnはposi/negaの略）

//各サーボの直立デフォルト値(KRS)
int idl_n[15];//idlのニュートラル補正値（nはneutralの略）
int idr_n[15];//idrのニュートラル補正値（nはneutralの略）

//各サーボのポジション値（中央値を0とした時の増減合計値）
float idl_d[15];//idlの増減分（d）
float idr_d[15];//idrの増減分（d）


//-------------------------------------------------------------------------
//---- 各 種 モ ー ド 設 定 0:OFF, 1:ON [S-4] -------------------------------
//-------------------------------------------------------------------------
bool trim_adjust = 0; //トリムモードのオンオフ、起動時に下記の設定値で静止させたい時は1
bool monitor_src = 0; //Teensyでのシリアル表示:送信ソースデータ
bool monitor_send = 0; //Teensyでのシリアル表示:送信データ
bool monitor_resv = 0; //Teensyでのシリアル表示:受信データ
bool monitor_resv_check = 1; //Teensyでのシリアル表示:受信成功の可否
bool monitor_resv_error = 0; //Teensyでのシリアル表示:受信エラー率
bool monitor_all_error = 0; //Teensyでのシリアル表示:全経路の受信エラー率
bool monitor_rpy = 0; //Teensyでのシリアル表示:IMUからのrpy換算値
bool monitor_joypad = 0; //Teensyでのシリアル表示:リモコンのデータ

void setup() {
  //-------------------------------------------------------------------------
  //---- サ ー ボ 設 定 [S-5] ------------------------------------------------
  //-------------------------------------------------------------------------
  //各サーボのマウントありなし（1:サーボあり、0:サーボなし）
  idl_mt[0]  = 1; //腰ヨー
  idl_mt[1]  = 1; //左肩ピッチ
  idl_mt[2]  = 1; //左肩ロール
  idl_mt[3]  = 1; //左肘ヨー
  idl_mt[4]  = 1; //左肘ピッチ
  idl_mt[5]  = 1; //左股ヨー
  idl_mt[6]  = 1; //左股ロール
  idl_mt[7]  = 1; //左股ピッチ
  idl_mt[8]  = 1; //左膝ピッチ
  idl_mt[9]  = 1; //左足首ピッチ
  idl_mt[10] = 1; //左足首ロール
  idl_mt[11] = 0; //追加テスト用
  idl_mt[12] = 0; //追加テスト用
  idl_mt[13] = 0; //追加テスト用
  idl_mt[14] = 0; //追加テスト用
  idr_mt[0]  = 1; //頭ヨー
  idr_mt[1]  = 1; //右肩ピッチ
  idr_mt[2]  = 1; //右肩ロール
  idr_mt[3]  = 1; //右肘ヨー
  idr_mt[4]  = 1; //右肘ピッチ
  idr_mt[5]  = 1; //右股ヨー
  idr_mt[6]  = 1; //右股ロール
  idr_mt[7]  = 1; //右股ピッチ
  idr_mt[8]  = 1; //右膝ピッチ
  idr_mt[9]  = 1; //右足首ピッチ
  idr_mt[10] = 1; //右足首ロール
  idr_mt[11] = 0; //追加テスト用
  idr_mt[12] = 0; //追加テスト用
  idr_mt[13] = 0; //追加テスト用
  idr_mt[14] = 0; //追加テスト用

  //各サーボの内外回転プラマイ方向補正
  idl_pn[0]  = +1;//腰ヨー
  idl_pn[1]  = +1;//左肩ピッチ
  idl_pn[2]  = +1;//左肩ロール
  idl_pn[3]  = +1;//左肘ヨー
  idl_pn[4]  = +1;//左肘ピッチ
  idl_pn[5]  = +1;//左股ヨー
  idl_pn[6]  = +1;//左股ロール
  idl_pn[7]  = +1;//左股ピッチ
  idl_pn[8]  = +1;//左膝ピッチ
  idl_pn[9]  = +1;//左足首ピッチ
  idl_pn[10] = +1;//左足首ロール
  idl_pn[11] = +1;//追加テスト用
  idl_pn[12] = +1;//追加テスト用
  idl_pn[13] = +1;//追加テスト用
  idl_pn[14] = +1;//追加テスト用
  idr_pn[0]  = +1;//頭ヨー
  idr_pn[1]  = +1;//右肩ピッチ
  idr_pn[2]  = +1;//右肩ロール
  idr_pn[3]  = +1;//右肘ヨー
  idr_pn[4]  = +1;//右肘ピッチ
  idr_pn[5]  = +1;//右股ヨー
  idr_pn[6]  = +1;//右股ロール
  idr_pn[7]  = +1;//右股ピッチ
  idr_pn[8]  = +1;//右膝ピッチ
  idr_pn[9]  = +1;//右足首ピッチ
  idr_pn[10] = +1;//右足首ロール
  idr_pn[11] = +1;//追加テスト用
  idr_pn[12] = +1;//追加テスト用
  idr_pn[13] = +1;//追加テスト用
  idr_pn[14] = +1;//追加テスト用

  //各サーボの直立デフォルト値　(KRS値  0deg=7500, +-90deg=7500+-2667  KRS値=deg/0.03375)
  //直立状態になるよう、具体的な数値を入れて現物調整する
  idl_n[0]  = 0;//腰ヨー
  idl_n[1]  = -70; //左肩ピッチ
  idl_n[2]  = -2700 ; //左肩ロール
  idl_n[3]  = 0;//左肘ヨー
  idl_n[4]  = 2666; //左肘ピッチ
  idl_n[5]  = 0; //左股ヨー
  idl_n[6]  = 0; //左股ロール
  idl_n[7]  = -40; //左股ピッチ
  idl_n[8]  = -1720; //左膝ピッチ
  idl_n[9]  = -600; //左足首ピッチ
  idl_n[10] = -20; //左足首ロール
  idl_n[11] = 0; //追加テスト用
  idl_n[12] = 0; //追加テスト用
  idl_n[13] = 0; //追加テスト用
  idl_n[14] = 0; //追加テスト用
  idr_n[0]  = 0; //頭ヨー
  idr_n[1]  = 0; //右肩ピッチ
  idr_n[2]  = -2650; //右肩ロール
  idr_n[3]  = 0;//右肘ヨー
  idr_n[4]  = 2666; //右肘ピッチ
  idr_n[5]  = 0;//右股ヨー
  idr_n[6]  = 50;//右股ロール
  idr_n[7]  = -100;//右股ピッチ
  idr_n[8]  = -1700; //右膝ピッチ
  idr_n[9]  = -600; //右足首ピッチ
  idr_n[10] = -70; //右足首ロール
  idr_n[11] = 0;//追加テスト用
  idr_n[12] = 0;//追加テスト用
  idr_n[13] = 0;//追加テスト用
  idr_n[14] = 0;//追加テスト用
  
  //入出力ピンのモード設定
  pinMode(ERR_LED, OUTPUT);//通信ディレイが生じたら点灯するLED（デフォルトはT2ピン）
  
  Serial.begin(SERIAL_PC);//シリアルモニター表示
  delay(100); merc = merc + 100; //ちょっと安定させるためのディレイ（要調整）
  krs_L.begin(); //サーボモータの通信初期設定。Serial2
  krs_R.begin(); //サーボモータの通信初期設定。Serial3
  if (ICS3_MOUNT == 1) {
    krs_3.begin(); //サーボモータの通信初期設定。Serial1
  }
  delay(100); merc = merc + 100; //ちょっと安定させるためのディレイ（要調整）

  //ESP32との通信用にSPI_MASTERを開始
  TsyDMASPI0.begin(SS, SPISettings(6000000, MSBFIRST, SPI_MODE3)); //moved from "void loop(){}"

  //SDカードの初期化

  //配列のリセット
  memset(s_merdim.bval, 0, MSG_BUFF);//配列要素を0でリセット
  memset(r_merdim.bval, 0, MSG_BUFF);//配列要素を0でリセット
  memset(s_packet.bval, 0, MSG_BUFF);//配列要素を0でリセット
  memset(r_packet.bval, 0, MSG_BUFF);//配列要素を0でリセット
  memset(idl_d, 0, 15); //配列要素を0でリセット
  memset(idr_d, 0, 15); //配列要素を0でリセット

  //I2CのSETUP
  if (IMU_MOUNT == 1) {
    setupMPU();
  }

  //変数の設定
  YAW_ZERO = 0;
  s_merdim.sval[0] = MSG_SIZE ;//(トップコマンド）

  //起動時のディレイ用mercちょい足し(サーボ起動待ち用)
  merc = merc + 3000;
}



//-------------------------------------------------------------------------
//---- 関 数 各 種  --------------------------------------------------------
//-------------------------------------------------------------------------

// ■ サーボトリム調整 サーボオンで直立静止-------------------------------------
void trimadjustment() {
  while (true) {
    for (int i = 0; i < 15; i++) {
      if (idl_mt[i] == 1) {
        krs_L.setPos(i, 7500 + (idl_n[i]*idl_pn[i]));
      }
      if (idr_mt[i] == 1) {
        krs_R.setPos(i, 7500 + (idr_n[i]*idr_pn[i]));
      }
      delayMicroseconds(2);
    }
    delay(100);
    Serial.println("Trim adjst mode.");
  }
}

// ■ degreeをKRS値に変換 -----------------------------------------------------
int Deg2Krs(float degree, int id_n) { //degreeにはidl_d[i] * idl_pn[i]、id_nにはidl_n[i]を入れる(左の場合は左半身系)
  float x = 7500 + id_n + (degree / 0.03375); //floatの小数点以下を四捨五入して整数化
  //ちなみにこの計算だと0.02度ぐらいからサーボ値には反映される(=0.59で1に繰り上がる)
  if (x > 11500) {
    x = 11500;
  } else if (x < 3500) {
    x = 3500;
  }
  return x ;
}

// ■ KRSをdegree値に変換 -----------------------------------------------------
float Krs2Deg(int krs, float n, float pn) { //KRS値のほか idl_n[i], idl_pn[i] を入れる(右の場合はidr系)
  float x = (krs - 7500 - n) * 3.375 * pn;//新
  x = x / 100; //小数点以下2桁で取得する　
  return x;
}

// ■ floatを100倍してshortに収める 限界を超えたら限界値張り付き short型で返す-------
short float2HFshort(float val) {// float to Hundredfold short
  double x = val * 100; //floatの小数点以下を四捨五入して整数化
  if (x > 32767) {
    x = 32767;
  } else if (x < -32768) {
    x = -32768;
  }
  int y = (int)round(x);
  return (short)y;
}

// ■ IMUの初期設定 ------------------------------------------------------------
void setupMPU() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-1745);
  mpu.setYAccelOffset(-1034);
  mpu.setZAccelOffset(966);
  mpu.setXGyroOffset(176);
  mpu.setYGyroOffset(-6);
  mpu.setZGyroOffset(-25);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print("DMP Initialization failed.");
  }
}

// ■ IMUのDMP推定値取得 ----------------------------------------------------------
void getYawPitchRoll() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // 最新のIMU情報を取得
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    ROLL = ypr[2] * 180 / M_PI;
    PITCH = ypr[1] * 180 / M_PI;
    YAW = (ypr[0] * 180 / M_PI) - YAW_ZERO;

    if (monitor_rpy == 1) { //Teensyでのシリアル表示:IMUからのrpy換算値
      Serial.print("[Roll, Pitch, Yaw] ");
      Serial.print(ROLL);
      Serial.print(", ");
      Serial.print(PITCH);
      Serial.print(", ");
      Serial.println(YAW);
    }
  }
}

// ■ ヨー軸の原点リセット --------------------------------------------------------
void setyaw() {
  YAW_ZERO = ypr[0] * 180 / M_PI;
  s_merdim.sval[0] = MSG_SIZE;
}

// ■ 全サーボオフ ---------------------------------------------------------------
void servo_all_off() {
  for (int h = 0; h < 5; h++) {
    for (int i = 0; i < 15; i++) {
      if (idl_mt[i] == 1) {
        krs_L.setFree(i);
      }
      if (idr_mt[i] == 1) {
        krs_R.setFree(i);
      }
      delayMicroseconds(2);
    }
  }
  delay(100);
  Serial.println("All servos off.");
}


// ■ JOYPAD処理 ---------------------------------------------------------------
void joypad_read() {
  if (JOYPAD_MOUNT == 3) {//KRR5FH(KRC-5FH)をICS_R系に接続している場合
    joypad_framecount ++;
    if (joypad_framecount >= JOYPAD_FRAME) {
      unsigned short buttonData;
      buttonData = krs_R.getKrrButton();
      delayMicroseconds(2);
      if (buttonData != KRR_BUTTON_FALSE) //ボタンデータが受信できていたら
      {
        button_1 = buttonData;
        if (monitor_joypad) {
          Serial.print("[Button] ");
          Serial.println(button_1);//ボタンデータを表示
        }
      }
      joypad_framecount = 0;
    }
  }
}
//-------------------------------------------------------------------------
//---- メ　イ　ン　ル　ー　プ ------------------------------------------------
//-------------------------------------------------------------------------

void loop() {

  //---- < 1 > センサー類読み取り ----------------------------------------------

  // [1-1] IMUの値を取得
  if (IMU_MOUNT == 1) {
    if (merc % IMU_FREQ == 0) {// IMU_FREQ(ms)ごとに読み取り
      getYawPitchRoll();
    }
  }

  //---- < 2 > コントローラの読み取り 動作の設定 ---------------------------------

  // [2-1] コントローラの値を取得
  if (JOYPAD_MOUNT != 0) {//ジョイパッドが接続設定されているかを判定
    joypad_read();
  }

  //---- < 3 > Teensy内部で位置制御する場合の処理 --------------------------------

  // [3-1] トップコマンドの判定によりこの工程の実行orスキップを分岐
  //デフォルトはMeridim配列数である90

  //コマンド90: サーボオン 通常動作(3-1-90)

  //コマンド0: 全サーボ脱力(3-1-0)

  //コマンド1: サーボオン 通常動作(3-1-1)

  //コマンド2: IMUのヨー軸設定(3-1-2)
  if (r_merdim.sval[0] == UPDATE_YAW_CENTER) {
    setyaw();
  }

  //コマンド3: トリムモードがオンもしくはコマンド3の時はループ
  if ((trim_adjust == 1) or (r_merdim.sval[0] == ENTER_TRIM_MODE)) {
    trimadjustment();
  }

  // [3-2] 前回のラストに読み込んだサーボ位置をサーボ配列に書き込む
  for (int i = 0; i < 15; i++) {
    s_KRS_servo_pos_L[i] = Deg2Krs(float(r_merdim.sval[i * 2 + 21]) / 100 * idl_pn[i], idl_n[i]); //
    s_KRS_servo_pos_R[i] = Deg2Krs(r_merdim.sval[i * 2 + 51] / 100 * idr_pn[i], idr_n[i]); //
  }

  // [3-3] Teensyによる次回動作の計算

  // [3-4] センサーデータによる動作へのフィードバック加味

  // [3-5] 移動時間の決定

  // [3-6] Teensy内計算による次回動作をMeridim配列に書き込む

  //---- < 4 > サーボコマンドの書き込み ------------------------------------------

  // [4-1] Meridim配列をサーボ命令に変更

  // [4-2] サーボコマンドの配列に書き込み

  // [4-3] サーボデータのICS送信および返り値を取得

  for (int i = 0; i < 11; i ++) {//接続したサーボの数だけ繰り返す。最大は15
    idl_d[i] = 0;
    if (idl_mt[i] == true) {
      if (r_merdim.sval[(i * 2) + 20] == 1) {//受信配列のサーボコマンドが1ならPos指定
        k = krs_L.setPos(i, s_KRS_servo_pos_L[i]);
      } else
      {
        k = krs_L.setFree(i);//1以外ならとりあえずサーボを脱力し位置を取得。手持ちの最大は15
      }
      idl_d[i] = Krs2Deg(k, idl_n[i], idl_pn[i]);
    }
    delayMicroseconds(2);
  }

  for (int i = 0; i < 11; i ++) {//接続したサーボの数だけ繰り返す。最大は15
    idr_d[i] = 0;
    if (idr_mt[i] == true) {
      if (r_merdim.sval[(i * 2) + 50] == 1) {//受信配列のサーボコマンドが1ならPos指定
        k = krs_R.setPos(i, s_KRS_servo_pos_R[i]);
      } else
      {
        k = krs_R.setFree(i);//1以外ならとりあえずサーボを脱力し位置を取得
      }
      idr_d[i] = Krs2Deg(k, idr_n[i], idr_pn[i]);
    }
    delayMicroseconds(2);
  }

  //---- < 5 > 送信用のMeridim配列を作成する -------------------------------------

  // [5-1] マスターコマンドを配列に格納
  //s_merdim.sval[0] = MSG_SIZE ;//デフォルトのマスターコマンドは配列数

  // [5-2] 移動時間を配列に格納
  //s_merdim.sval[1] = 10 ;//(移動時間）

  // [5-3] センサー値を配列に格納
  //s_merdim.sval[2] = (short)acc_x * 100 ; //IMU_gyro_x
  //s_merdim.sval[3] = (short)acc_y * 100 ; //IMU_gyro_y
  //s_merdim.sval[4] = (short)acc_z * 100 ; //IMU_gyro_z
  //s_merdim.sval[5] = (short)gyro_x * 100 ; //IMU_acc_x
  //s_merdim.sval[6] = (short)gyro_y * 100 ; //IMU_acc_y
  //s_merdim.sval[7] = (short)gyro_z * 100 ; //IMU_acc_z
  s_merdim.sval[8] = 0 ;//IMU
  s_merdim.sval[9] = 0 ;//IMU
  s_merdim.sval[10] = 0 ;//IMU
  //s_merdim.sval[11] = (short)temp ;//IMU
  s_merdim.sval[12] = float2HFshort(ROLL) ;//Madgwick_roll
  s_merdim.sval[13] = float2HFshort(PITCH)  ;//Madgwick_pitch
  s_merdim.sval[14] = float2HFshort(YAW) ;//Madgwick_yaw

  // [5-4] サーボIDごとにの現在位置もしくは計算結果を配列に格納
  for (int i = 0; i < 15; i++) {
    s_merdim.sval[i * 2 + 20] = 0; //仮にここでは各サーボのコマンドを脱力&ポジション指示(0)に設定
    s_merdim.sval[i * 2 + 21] = float2HFshort(idl_d[i]); //仮にここでは最新のサーボ角度degreeを格納
  }
  for (int i = 0; i < 15; i++) {
    s_merdim.sval[i * 2 + 50] = 0; //仮にここでは各サーボのコマンドを脱力&ポジション指示(0)に設定
    s_merdim.sval[i * 2 + 51] = float2HFshort(idr_d[i]); //仮にここでは最新のサーボ角度degreeを格納
  }

  // [5-5] カスタムデータを配列格納

  // [5-6] チェックサムを計算
  checksum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i ++) {
    checksum += s_merdim.sval[i];
  }
  checksum = ~checksum & 0xffff;
  s_merdim.sval[MSG_SIZE - 1] = checksum;

  // [5-7] 送信データのSPIバッファへのバイト型書き込み
  for (int i = 0; i < MSG_BUFF; i++) {
    s_packet.bval[i] = s_merdim.bval[i];
  }


  //---- < 6 > ESP32とのISPによる送受信処理 --------------------------------------

  // [6-1] シリアルモニタ表示（送信データ）
  if (monitor_src == 1) {
    Serial.print("   [Src] ");
    for (int i = 0; i < MSG_SIZE; i++) {
      Serial.print(int (s_merdim.sval[i]));
      Serial.print(",");
    }
    Serial.println();
  }
  if (monitor_send == 1) {
    Serial.print("  [Send] ");
    for (int i = 0; i < MSG_BUFF; i++) {
      Serial.print(int (s_packet.bval[i]));
      Serial.print(",");
    }
    Serial.println();
  }

  // [6-2] ESP32へのSPI送信の実行
  if (ESP32_MOUNT == 1) {
    TsyDMASPI0.transfer(s_packet.bval, r_packet.bval, MSG_BUFF);

    // [6-3] ESP32からのSPI受信データチェックサム確認と成否のシリアル表示
    int checksum = 0;
    for (int i = 0; i < MSG_SIZE - 1; i++) {
      checksum += r_packet.sval[i];
    }
    checksum = ~checksum & 0xffff;
    spi_trial ++;

    if ((short)r_packet.sval[MSG_SIZE - 1] == (short)checksum)//チェックがOKならバッファから受信配列に転記
    {
      if (monitor_resv_check == 1) {
        Serial.println("Rvok! ");//受信OKのシリアル表示
      }
      for (int i = 0; i < MSG_SIZE; i++) {
        r_merdim.sval[i] = r_packet.sval[i];
      }
      spi_ok ++;
    } else
    {
      if (monitor_resv_check == 1) {
        Serial.println("RvNG****");//受信のシリアル表示
      }
    }
    if (monitor_resv_error == 1) {
      if (spi_trial % 200 == 0) { //エラー率の表示
        Serial.print("error rate ");
        Serial.print(float(spi_trial - spi_ok) / float(spi_trial) * 100);
        Serial.print(" %  ");
        Serial.print(spi_trial - spi_ok);
        Serial.print("/");
        Serial.println(spi_trial);
      }
    }

    // [6-4] シリアルモニタ表示（受信データ）
    if (monitor_resv == 1) {
      Serial.print("  [Resv] ");
      for (int i = 0; i < MSG_SIZE; i++) {
        Serial.print(int (r_merdim.sval[i]));
        Serial.print(",");
      }
      Serial.println();
    }
  }


  //---- < 7 > フレーム終端処理 -------------------------------------------------

  // [7-1] この時点で１フレーム内に処理が収まっていない時の処理
  curr = (long)millis(); // 現在時刻を更新
  if (curr > merc) { // 現在時刻がフレーム管理時計を超えていたらアラートを出す
    //シリアルに遅延msを表示
    Serial.println("*** processing delay :");
    Serial.println(curr - merc);
    digitalWrite(ERR_LED, HIGH);//処理落ちが発生していたらLEDを点灯
  }
  else {
    digitalWrite(ERR_LED, LOW);//処理が収まっていればLEDを消灯
  }

  // [7-2] この時点で時間が余っていたら時間消化。時間がオーバーしていたらこの処理を自然と飛ばす。
  curr = (long)millis();
  curr_micro = (long)micros(); // 現在時刻を取得
  //Serial.println(merc * 1000 - curr_micro); //詳細な残り時間をμ秒単位でシリアル表示
  while (curr < merc) {
    curr = (long)millis();
  }

  // [7-3]フレーム管理時計mercのカウントアップ
  merc = merc + frame_ms;//フレーム管理時計を1フレーム分進める
  framecount = framecount + framecount_diff;//サインカーブ動作用のフレームカウントをいくつずつ進めるかをここで設定。
}
