/**************************************************************
*   arduino_ros
*   高縄祐樹
*   (改)濵口和也
*   rosで足回りを動かすSerial_node 総数596行
**************************************************************/
#define USE_USBCON

#include <DueTimer.h>

#include <ros.h>
#include <nemcon/switch_in.h>
#include <std_msgs/Int32.h>
#include <nemcon/motor.h>
#include <deadreckoning/enc.h>
#include <nemcon/object_in.h>

#define SERIAL_FRAMEDELIMITER   0x7e
#define SERIAL_ESCAPECHAR       0x7d 

#define sec 0.01                    //繰り返す秒数
#define wheel 47.48                //タイヤの直径もしくは、タイヤの直径＊ギヤ比(mm)  1周:0.149m
#define Pi 3.1415926535897932       //円周率

/*ロータリーピン番号*///////////////////////////////////////////////////////////////////
#define pin_XA 23 //ROT1
#define pin_XB 22 //ROT1
#define pin_YA 25 //ROT2
#define pin_YB 24 //ROT2
/*対物センサ番号*///////////////////////////////////////////////////////////////////
#define pin_objR 27 //対物センサ右
#define pin_objL 26 //対物センサ左
/*モーター番号*////////////////////////////////////////////////////////////////////////
#define FR_N 0x01
#define FL_N 0x02
#define RR_N 0x04
#define RL_N 0x03
#define TH_N 0x05

bool RESET = false;
bool first = false;

/* プロトタイプ宣言 *//////////////////////////////////////////////////////////////////
void Serial_putc(unsigned char, unsigned char);
unsigned int HEX_conversion(int DEC_sokudo);

//////////////////////////////////////////////////////////////////////////////////////
ros::NodeHandle  nh;
deadreckoning::enc enc_msg;
nemcon::object_in obj_msg;
ros::Publisher pub_enc("enc", &enc_msg);
ros::Publisher pub_obj("object_in", &obj_msg);

////////////////////////////////////////////////////////////////////////////////////////////////
void motor(const nemcon::motor& msg)
{
  unsigned int sokudo_FR, sokudo_FL, sokudo_RR, sokudo_RL;
  sokudo_FR = HEX_conversion(msg.motor_FR);
  sokudo_FL = HEX_conversion(msg.motor_FL);
  sokudo_RR = HEX_conversion(msg.motor_RR);
  sokudo_RL = HEX_conversion(msg.motor_RL);
  Serial_putc(FR_N, sokudo_FR);
  Serial_putc(FL_N, sokudo_FL);
  Serial_putc(RR_N, sokudo_RR);
  Serial_putc(RL_N, sokudo_RL);
}
ros::Subscriber<nemcon::motor> motor_msg("motor", motor); //ROSにmoter_FRっていうtopicが来たらmoter_FR_cbを実行するように教

////////////////////////////////////////////////////////////////////////////////////////////////
void switch_cb(const nemcon::switch_in& msg)
{
  RESET = msg.RESET;

  if(msg.RESET == true)
  {
    first = false;
  }
}
ros::Subscriber<nemcon::switch_in> switch_msg("switch", switch_cb);

//////////////////////////* Encoder用変数定義 *////////////////////////////////////////////////////
float cir = Pi * wheel;         //circumference(円周)進むと360パルス
float pulse = (360 * 1000) / cir;     //1m進んだ時のパルス数の計算
float repeat = sec * 1000000;       //マイクロ秒にする

//////////////////////////////interrupt関係////////////////////////////////////////////////////////
void set_up2(int pinC, int pinD);
void loop2(int pinC, int pinD);
void sokudo(void);
void rotary_changedPin_X(void);
void rotary_changedPin_Y(void);
void obj();

///////////////////////////////////////////enc_X///////////////////////////////////////////////////////
float rot_count_X = 0.0;                     // 一周を720countとして状態が4ステップなので2880ステップに拡張
float pre_pulse_X = 0.0, cur_pulse_X = 0.0;   //pre：前    cur = current：現在
float sokudo_X = 0.0;
int direction_X = 0;
int parse_X;

///////////////////////////////////////////enc_Y//////////////////////////////////////////////////////
float rot_count_Y = 0.0;                    // 一周を720countとして状態が4ステップなので2880ステップに拡張
float pre_pulse_Y = 0.0, cur_pulse_Y = 0.0;  //pre：前    cur = current：現在
float sokudo_Y = 0.0;
int direction_Y = 0;
int parse_Y;

///////////////////////////////////////////////* main *///////////////////////////////////////////////////
void setup()
{
  Serial2.begin(9600);
  nh.initNode();
  nh.subscribe(motor_msg);
  nh.subscribe(switch_msg);
  nh.advertise(pub_enc);
  nh.advertise(pub_obj);
  pinMode(pin_objR,INPUT);
  pinMode(pin_objL,INPUT);
  set_up2(pin_XA, pin_XB);
  set_up2(pin_YA, pin_YB);
  attachInterrupt(pin_XA,rotary_changedPin_X,CHANGE);
  attachInterrupt(pin_XB,rotary_changedPin_X,CHANGE);
  attachInterrupt(pin_YA,rotary_changedPin_Y,CHANGE);
  attachInterrupt(pin_YB,rotary_changedPin_Y,CHANGE);
  Timer1.attachInterrupt(sokudo);
  Timer1.start(repeat);
  nh.loginfo("Startup complete");
}

void loop()
{
  if(RESET)
  {
    reset();
  }
  else
  {
    loop2(pin_XA, pin_XB);
    loop2(pin_YA, pin_YB);
    obj();
  }
  nh.spinOnce();
  delay(1);
}
///////////////////////////////////////////////////////* main 終了*///////////////////////////////////////////////////
