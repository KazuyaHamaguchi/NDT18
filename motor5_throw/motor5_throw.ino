/**************************************************************
*   ServiceServer_motor
*   高縄祐樹
*   rosのトピック通信で投射を行う
**************************************************************/
#define USE_USBCON

#include <ros.h>
#include <std_msgs/Int8.h>
#include <nemcon/switch_in.h>

#define SERIAL_FRAMEDELIMITER   0x7e
#define SERIAL_ESCAPECHAR       0x7d

/* プロトタイプ宣言 *//////////////////////////////////////////////////////////////////
void Serial_putc(unsigned char id, unsigned char data);
unsigned int HEX_conversion(int DEC_sokudo);
void throw_on_2(void);
void throw_on_3(void);
void throw_on_4(void);
void Receive_Setup_1(void);
void Receive_Setup(void);
void Receive_Starte(void);
void Receive_Starte_1(void);
void throw_on_setup(void);
void throw_on_END(void);
void Receive_Setup_2(void);
void rotary_changedPin_R(void);
void rotary_changedPin(void);
void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t mSec);
float pid_smart(float sensor_val, float target_val, float KP, float KI, float KD, float KB, int AW);
void Res(void);
//////////////////////////////////////////////////////////////////////////////////////
ros::NodeHandle  nh;
std_msgs::Int8 Throw_on_msg;
ros::Publisher pub_Throw_on("Throw_on", &Throw_on_msg);

/*エンコーダー関係*////////////////////////////////////////////////////////////////////
#define SERIAL_BUFFERSIZE       5
#define SERIAL_FRAMEDELIMITER   0x7e
#define SERIAL_ESCAPECHAR       0x7d
#define pinA 23
#define pinB 22
#define pin_A 24
#define pin_B 25
#define pin_RS 52
#define SERIAL_ID 0x10
#define setpow 0x00       //調整出力設定
#define DELTA_T 0.005
#define airPin  27
#define airPin_R 26
#define laserPin  28
#define MD_Num 0x05
#define MD_Num_R 0x06
#define DELTA 30  //アームの初期位置から受け取り位置までの角度

/* Encoder用変数定義 */
volatile float rot_count = 0, angle = 0;
int direction = 0;
int parse;
volatile float count, count_A, nspeed, Setspeed, Setangle, r = 0.45;//rはｍ表記で！
float angle_A = 0, angle_B = 0;
int flag_K = 0;

/* Encoder用変数定義 */
volatile float rot_count_R = 0, angle_R = 0, rot_count_rpm = 0;
int direction_R = 0;
int parse_R;
volatile float count_R, count_AR, nspeed_R;

/* PID初期設定 */
static float diff[2];
static float integral;
static float aw_bc = 0, aw_bc_old = 0;

/*その他必要変数*/
bool accel;
char data = 0;                            //　シリアル通信　保存用変数
int cir = 0;                              //  ハンド回転数　保存用変数
int buttonState = 0;
static int Output;

bool RESET = false;
bool first = false;

double TZ3_deg = /*25.67*/20.67;    //TZ3の角度調整

///////////////////////////////////////////////////////////////////////////////////////////////

void TC3_Handler(void) {                                 // 角速度から周速度を算出 : km/h
  TC_GetStatus(TC1, 0);
  count = rot_count;
  count_A = count - count_A;
  nspeed = ((r * count_A * 3.141592653589793) / (180 * DELTA_T));
  count_A = count;
  count_R = rot_count_rpm;
  count_AR = count_R - count_AR;
  nspeed_R = ((r * count_AR * 3.141592653589793) / (180 * DELTA_T));
  count_AR = count_R;
}
////////////////////////////////////////////////////////////////////////////////////////////////
void Throw_on_cb( const std_msgs::Int8& msg) //std_msgsのInt8をmsgとした
{
  if(msg.data == 111)//TZ3
  {
    flag_K = 0;
    throw_on_2();
    nh.loginfo("Throw_on_3 ==== Finish ====");
    Throw_on_msg.data = -111;
    pub_Throw_on.publish(&Throw_on_msg);
    Receive_Setup_1();
    throw_on_END();
    Throw_on_msg.data = -100;
    pub_Throw_on.publish(&Throw_on_msg);
  }
  else if(msg.data == 11)//TZ2
  {
    flag_K = 0;
    throw_on_3();
    nh.loginfo("Throw_on_2 ==== Finish ====");
    Throw_on_msg.data = -11;
    pub_Throw_on.publish(&Throw_on_msg);
    Receive_Setup_1();
    throw_on_END();
    Throw_on_msg.data = -100;
    pub_Throw_on.publish(&Throw_on_msg);
  }
  else if(msg.data == 1)//TZ1
  {
    flag_K = 0;
    throw_on_4();
    nh.loginfo("Throw_on_1 ==== Finish ====");
    Throw_on_msg.data = -1;
    pub_Throw_on.publish(&Throw_on_msg);
    Receive_Setup_1();
    throw_on_END();
    Throw_on_msg.data = -100;
    pub_Throw_on.publish(&Throw_on_msg);
  }
  else if(msg.data == 2)
  {
    digitalWrite(airPin, LOW);//閉める
    nh.loginfo("==== CLOSE ====");
    Serial_putc(MD_Num, 0x00);
    delay(5);
    Serial_putc(MD_Num_R, 0x00);
    Throw_on_msg.data = -2;
    pub_Throw_on.publish(&Throw_on_msg);
  }
  else if(msg.data == 3)
  {
    digitalWrite(airPin, HIGH);//開ける
    nh.loginfo("==== OPEN ====");
    Serial_putc(MD_Num, 0x00);
    delay(5);
    Serial_putc(MD_Num_R, 0x00);
    Throw_on_msg.data = -3;
    pub_Throw_on.publish(&Throw_on_msg);
  }
  else if(msg.data == 4)//投射アームを受け渡し位置まで動かす
  {
    flag_K = 0;
    throw_on_setup();
    Throw_on_msg.data = -4;
    pub_Throw_on.publish(&Throw_on_msg);
  }
  else if(msg.data == 10)
  {
    Receive_Starte();
    Serial_putc(MD_Num, 0x00);
    delay(5);
    Serial_putc(MD_Num_R, 0x00);
    nh.loginfo("Receive ==== Finish ====");
    Throw_on_msg.data = -10;
    pub_Throw_on.publish(&Throw_on_msg);
  }
  else if(msg.data == 20)
  {
    digitalWrite(airPin_R, LOW);//閉める
    nh.loginfo("==== CLOSE_R ====");
    Serial_putc(MD_Num, 0x00);
    delay(5);
    Serial_putc(MD_Num_R, 0x00);
    Throw_on_msg.data = -20;
    pub_Throw_on.publish(&Throw_on_msg);
  }
  else if(msg.data == 30)
  {
    digitalWrite(airPin_R, HIGH);//開ける
    nh.loginfo("==== OPEN_R ====");
    Serial_putc(MD_Num, 0x00);
    delay(5);
    Serial_putc(MD_Num_R, 0x00);
    Throw_on_msg.data = -30;
    pub_Throw_on.publish(&Throw_on_msg);
  }
  else if(msg.data == 40)
  {
    flag_K = 0;
    //digitalWrite(airPin_R, HIGH);
    delay(5);
    //digitalWrite(airPin, HIGH);
    delay(5);
    while(1)
    {
      nh.spinOnce();
      if(digitalRead(laserPin) == LOW)
      {
        nh.loginfo("OK");
        delay(700);
        digitalWrite(airPin_R, LOW);
        break;
      }
      if(flag_K == 1)
      {
        nh.loginfo("OK_RS");
        flag_K = 0;
        Serial_putc(MD_Num, 0x00);
        delay(5);
        Serial_putc(MD_Num_R, 0x00);
        delay(5);
        digitalWrite(airPin, LOW);
        delay(5);
        digitalWrite(airPin_R, LOW);
        delay(5);
        break;
      }
    }
    Throw_on_msg.data = -40;
    pub_Throw_on.publish(&Throw_on_msg);
  }
  else if(msg.data == 41)//受け渡し
  {
    flag_K = 0;
    Receive_Starte();
    nh.loginfo("Receive ==== Finish ====");
    Throw_on_msg.data = -41;
    pub_Throw_on.publish(&Throw_on_msg);
    
  }
  else if(msg.data == 44)//受け渡しアームを判断の邪魔にならないようにする
  {
    flag_K = 0;
    Receive_Starte_1();
     nh.loginfo("Receive_Starte_1 ==== Finish ====");
    Throw_on_msg.data = -44;
    pub_Throw_on.publish(&Throw_on_msg);
  }
  else if(msg.data == 43)//最初に受け渡しアームを受け渡し位置まで動かす
  {
    flag_K = 0;
    Receive_Setup();
    Throw_on_msg.data = -43;
    pub_Throw_on.publish(&Throw_on_msg);
  }
  else if(msg.data == 45)//受け渡しアームを受け渡し位置まで動かす
  {
    flag_K = 0;
    Receive_Setup_1();
    Throw_on_msg.data = -45;
    pub_Throw_on.publish(&Throw_on_msg);
  }
  else
  {
    nh.loginfo("ERROR");
    Serial_putc(MD_Num, 0x00);
    delay(5);
    Serial_putc(MD_Num_R, 0x00);
    /*Throw_on_msg.data = angle_A;
    pub_Throw_on.publish(&Throw_on_msg);*/
  }
}
ros::Subscriber<std_msgs::Int8> sub_Throw_on("Throw_on_1", Throw_on_cb); //ROSにThrow_on_1っていうtopicが来たらThrow_on_cbを実行するように教


void Reset_cb( const std_msgs::Int8& msg) //std_msgsのInt8をmsgとした
{

}
ros::Subscriber<std_msgs::Int8> sub_Reset("Reset", Reset_cb); //ROSにResetっていうtopicが来たらReset_cbを実行するように教

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

///////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial_putc(MD_Num, 0x80);
  delay(5);
  Serial_putc(MD_Num_R, 0x80);
  delay(5);
  int current_a;
  int current_b;
  //Serial.begin( 57600 );
  Serial2.begin( 9600 );
  pinMode(pin_RS, INPUT);
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode(pin_A, INPUT);
  pinMode(pin_B, INPUT);
  pinMode(airPin, OUTPUT);
  pinMode(airPin_R, OUTPUT);
  pinMode(laserPin, INPUT_PULLUP);
  /* LED サンプル用 */
  pinMode(13, OUTPUT);
  current_a = digitalRead(pinA);
  current_b = digitalRead(pinB);
  /* rotary振り分け関数 */
  if (( current_a == 0 ) && ( current_b == 0 ))   // A,Bとも０
  {
    parse = 0;
  }
  if (( current_a == 1 ) && ( current_b == 0 ))   // A=1 B = 0 CCW
  {
    parse = 1;
  }
  if (( current_a == 1 ) && ( current_b == 1 ))   // A,Bとも０
  {
    parse = 2;
  }
  if (( current_a == 0 ) && ( current_b == 1 ))   // A,Bとも０
  {
    parse = 3;
  }
  current_a = digitalRead(pin_A);
  current_b = digitalRead(pin_B);
  /* rotary振り分け関数 */
  if (( current_a == 0 ) && ( current_b == 0 ))   // A,Bとも０
  {
    parse_R = 0;
  }
  if (( current_a == 1 ) && ( current_b == 0 ))   // A=1 B = 0 CCW
  {
    parse_R = 1;
  }
  if (( current_a == 1 ) && ( current_b == 1 ))   // A,Bとも０
  {
    parse_R = 2;
  }
  if (( current_a == 0 ) && ( current_b == 1 ))   // A,Bとも０
  {
    parse_R = 3;
  }
  attachInterrupt(pinA, rotary_changedPin, CHANGE);
  attachInterrupt(pinB, rotary_changedPin, CHANGE);
  attachInterrupt(pin_A, rotary_changedPin_R, CHANGE);
  attachInterrupt(pin_B, rotary_changedPin_R, CHANGE);
  attachInterrupt(pin_RS, Res, LOW);
  startTimer(TC1, 0, TC3_IRQn, 5);//ms単位
  nh.initNode();
  nh.subscribe(switch_msg);
  nh.subscribe(sub_Throw_on);
  nh.advertise(pub_Throw_on);
  nh.loginfo("Startup complete");
}

void loop()
{
  if(RESET)
  {
    reset();
  }
  else;
  //Serial.println(angle_R);
  nh.spinOnce();
  delay(10);
}

