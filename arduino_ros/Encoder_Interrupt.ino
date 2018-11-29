#include<DueTimer.h>
unsigned long time_u, time_c;
void sokudo(void)
{
  float iEncoder1, iEncoder2,iEncoder3,iEncoder4;
  cur_pulse_X = rot_count_X;
  cur_pulse_Y = rot_count_Y;
  iEncoder1 = cur_pulse_X - pre_pulse_X;  //10msecで進んだ距離(m)
  iEncoder2 = cur_pulse_Y - pre_pulse_Y;  //10msecで進んだ距離(m)
  sokudo_X = iEncoder1 / (pulse * sec);    //現在の速度(m/s)
  sokudo_Y = iEncoder2 / (pulse * sec);    //現在の速度(m/s)
  pre_pulse_X = cur_pulse_X;
  pre_pulse_Y = cur_pulse_Y;
}

void set_up2(int pinC, int pinD)
{
  int current_a;
  int current_b;
  pinMode(pinC,INPUT);
  pinMode(pinD,INPUT);
  current_a = digitalRead(pinC);
  current_b = digitalRead(pinD);
/* rotary振り分け関数 */
  if(pinC == pin_XA && pinD == pin_XB)
  {
    if(( current_a == 0 ) && ( current_b == 0 ))    // A,Bとも０
    {
      parse_X = 0;
    }
    if(( current_a == 1 ) && ( current_b == 0 ))    // A=1 B = 0 CCW
    {
      parse_X = 1;
    }
    if(( current_a == 1 ) && ( current_b == 1 ))    // A,Bとも０
    {
     parse_X = 2;
    }
    if(( current_a == 0 ) && ( current_b == 1 ))    // A,Bとも０  
    {
      parse_X = 3;
    }
  }
  else if(pinC == pin_YA && pinD == pin_YB)
  {
    if(( current_a == 0 ) && ( current_b == 0 ))    // A,Bとも０
    {
      parse_Y = 0;
    }
    if(( current_a == 1 ) && ( current_b == 0 ))    // A=1 B = 0 CCW
    {
      parse_Y = 1;
    }
    if(( current_a == 1 ) && ( current_b == 1 ))    // A,Bとも０
    {
     parse_Y = 2;
    }
    if(( current_a == 0 ) && ( current_b == 1 ))    // A,Bとも０  
    {
      parse_Y = 3;
    }
  }
}

void loop2(int pinC, int pinD)
{
  if(pinC == pin_XA && pinD == pin_XB)
  {
    enc_msg.distance_X = rot_count_X / pulse;
    enc_msg.speed_X = sokudo_X;
    pub_enc.publish(&enc_msg);
  }
  else if(pinC == pin_YA && pinD == pin_YB)
  {
    enc_msg.distance_Y = rot_count_Y / pulse;
    enc_msg.speed_Y = sokudo_Y;
    pub_enc.publish(&enc_msg);
  }
}
