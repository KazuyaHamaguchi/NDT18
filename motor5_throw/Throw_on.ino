void throw_on_2(void)       //TZ3
{
  float angle_cb = 0;
  rot_count = angle;
  cir = 0;
  count = 0;
  count_A = 0;
  nspeed = 0;
  diff[2] = 0;
  integral = 0;
  aw_bc = 0;
  aw_bc_old = 0;
    
  do{
    //pid_smart(nspeed, 15.0, 3.41, 4.2, 0.05, 0.05, 1);//TZ1,TZ2
    pid_smart(nspeed, 18.5, 3.66, 1.43, 0.0015, 1.45, 1);//TZ3 G1号用
    //pid_smart(nspeed, 17.5, 3.61, 0.98, 0.0065, 0.98, 1);//TZ3 G1号用
    //pid_smart(nspeed, 18.0, 3.61, 1.45, 0.0065, 1.45, 1);//TZ3 N1号用
    delay(4);                        //Serial_putcに移行する際の脱調防止
    Serial_putc(MD_Num, Output);
    if( cir > 6 && angle_A >= TZ3_deg)//TZ1 == angle_A 10 TZ2 == angle_A 10 TZ3 == angle_A 25 初期位置はアームの先端が地面(100g angle_A >= 23.35 && angle_A <= 23.55, 90g angle_A >= 24.80 && angle_A <= 25.00)
    {
      digitalWrite(airPin, HIGH);
      angle_cb = angle_A;
      break;
    }
    if(flag_K == 1)
    {
      nh.loginfo("OK");
      break;
    }
    cir = rot_count / 720;
    } while (1);// TZ2=2890angle <= 2894  angle <= 2939  ////angle<=1810////
    delay(3);                        //Serial_putcに移行する際の脱調防止
    TZ3_deg = TZ3_deg - 0.5;
    Serial_putc(MD_Num, 0x00);
}

void throw_on_3(void)       //TZ2
{
  float angle_cb = 0;
  rot_count = angle;
  cir = 0;
  count = 0;
  count_A = 0;
  nspeed = 0;
  diff[2] = 0;
  integral = 0;
  aw_bc = 0;
  aw_bc_old = 0;

  do{
    pid_smart(nspeed, 15.25, 3.41, 4.15, 0.05, 0.05, 1);//TZ1,TZ2
    //pid_smart(nspeed, 17.5, 3.61, 0.98, 0.0065, 0.98, 1);//TZ3 G1号用
    //pid_smart(nspeed, 18.0, 3.61, 1.45, 0.0065, 1.45, 1);//TZ3 N1号用
    delay(4);                        //Serial_putcに移行する際の脱調防止
    Serial_putc(MD_Num, Output);
    if( cir > 6 && angle_A >= 21.75 /*&& 19.70angle_A <= 20.20*/)//TZ1 == angle_A 10 TZ2 == angle_A 10 TZ3 == angle_A 25 初期位置はアームの先端が地面(100g angle_A >= 23.35 && angle_A <= 23.55, 90g angle_A >= 24.80 && angle_A <= 25.00)
    {
      digitalWrite(airPin, HIGH);
      angle_cb = angle_A;
      break;
    }
    if(flag_K == 1)
    {
      nh.loginfo("OK");
      break;
    }
    cir = rot_count / 720;
    } while (1);// TZ2=2890angle <= 2894  angle <= 2939  ////angle<=1810////
    delay(3);                        //Serial_putcに移行する際の脱調防止
    Serial_putc(MD_Num, 0x00);
}

void throw_on_4(void)         //TZ1
{
  float angle_cb = 0;
  rot_count = angle;
  cir = 0;
  count = 0;
  count_A = 0;
  nspeed = 0;
  diff[2] = 0;
  integral = 0;
  aw_bc = 0;
  aw_bc_old = 0;
    
  do{
    pid_smart(nspeed, 15.0, 3.41, 4.05, 0.05, 0.05, 1);//TZ1,TZ2
    //pid_smart(nspeed, 17.5, 3.61, 0.98, 0.0065, 0.98, 1);//TZ3 G1号用
    //pid_smart(nspeed, 18.0, 3.61, 1.45, 0.0065, 1.45, 1);//TZ3 N1号用
    delay(4);                        //Serial_putcに移行する際の脱調防止
    Serial_putc(MD_Num, Output);
    if( cir > 6 && angle_A >= 21.90/*angle_A >= 24.00 && angle_A <= 24.20*/)//TZ1 == angle_A 10 TZ2 == angle_A 10 TZ3 == angle_A 25 初期位置はアームの先端が地面(100g angle_A >= 23.35 && angle_A <= 23.55, 90g angle_A >= 24.80 && angle_A <= 25.00)
    {
      digitalWrite(airPin, HIGH);
      angle_cb = angle_A;
      break;
    }
    if(flag_K == 1)
    {
      nh.loginfo("OK");
      break;
    }
    cir = rot_count / 720;
    } while (1);// TZ2=2890angle <= 2894  angle <= 2939  ////angle<=1810////
    delay(3);                        //Serial_putcに移行する際の脱調防止
    Serial_putc(MD_Num, 0x00);
}

void throw_on_END(void)//投射終了後に投射アームを受け取り位置まで動かす
{
    while(1)
    {
      nh.spinOnce();
      if(nspeed == 0)
      {
        Serial_putc(MD_Num, 0x80);
        delay(5);
        break;
      }
      else;
      if(flag_K == 1)
      {
        nh.loginfo("OK");
        break;
      }
    }
    delay(300);
    while(1)
    {
      nh.spinOnce();
      Serial_putc(MD_Num, 0x01);
      delay(5);
      if(nspeed >= 1.5)
      {
        Serial_putc(MD_Num, 0x00);
        delay(5);
      }
      if(angle_A >= 270.0 && angle_A <= 290.9)//angle_A >= 289.0 && angle_A <= 290.0
      {
        Serial_putc(MD_Num, 0x80);
        delay(5);
        break;
      }
      if(flag_K == 1)
      {
        nh.loginfo("OK");
        break;
      }
    }
    while(1)
    {
      nh.spinOnce();
      Serial_putc(MD_Num, 0x01);
      delay(5);
      if(nspeed >= 0.3)
      {
        Serial_putc(MD_Num, 0x80);
        delay(5);
      }
      if(angle_A >= 291.0 && angle_A <= 292.0)//angle_A >= 289.0 && angle_A <= 290.0
      {
        Serial_putc(MD_Num, 0x80);
        delay(5);
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
}

void throw_on_setup(void)//投射アームを受け渡し位置まで動かす
{
    while(1)
    {
      nh.spinOnce();
      Serial_putc(MD_Num, 0x82);
      delay(5);
      if(nspeed <= -1.5)
      {
        Serial_putc(MD_Num, 0x80);
        delay(5);
      }
      if(angle_A <= -49.0)//angle_A >= 289.0 && angle_A <= 290.0
      {
        Serial_putc(MD_Num, 0x80);
        delay(5);
        break;
      }
      if(flag_K == 1)
      {
        nh.loginfo("OK");
        break;
      }
    }
   while(1)
    {
      nh.spinOnce();
      Serial_putc(MD_Num, 0x81);
      delay(5);
      if(nspeed <= -0.3)
      {
        Serial_putc(MD_Num, 0x80);
        delay(5);
      }
      if(angle_A <= -68.0)
      {
        Serial_putc(MD_Num, 0x80);
        delay(5);
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
}

