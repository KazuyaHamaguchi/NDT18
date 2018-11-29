void Receive_Starte_1(void)//受け渡しアームを判断の邪魔にならないようにする
{
  digitalWrite(airPin_R, LOW);
  delay(3);
  digitalWrite(airPin, HIGH);
  delay(3);
  while(1)
  {
    //Serial.println(nspeed);
    Serial_putc(MD_Num_R,0x8A);
    delay(5);
    if(nspeed_R <= -2.0)
    {
      Serial_putc(MD_Num_R, 0x00);
      delay(5);
    }
    else;
    if(angle_R <= -180)
    {
      Serial_putc(MD_Num_R, 0x80);
      delay(5);
      break;
    }
    else;
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
    nh.spinOnce();
  }
}

void Receive_Starte(void)//受け渡し
{
  while(1)
  {
    //Serial.println(nspeed);
    Serial_putc(MD_Num_R,0x87);
    delay(5);
    if(nspeed_R <= -0.4)
    {
      Serial_putc(MD_Num_R, 0x00);
      delay(5);
    }
    else;
    if(angle_R <= -297)
    {
      Serial_putc(MD_Num_R, 0x80);
      delay(5);
      break;
    }
    else;
    if(flag_K == 1)
    {
      nh.loginfo("OK");
      break;
    }
    nh.spinOnce();
  }
  delay(500);
  digitalWrite(airPin, LOW);
  delay(500);
  digitalWrite(airPin_R, HIGH);
  delay(500);
  if(angle_A <= 0)
  {
    while(1)
    {
      Serial_putc(MD_Num, 0x01);
      delay(5); 
      if(nspeed >= 1.5)
      {
        Serial_putc(MD_Num, 0x80);
        delay(5);
      }
      if(angle_A >= -20)
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
      nh.spinOnce();
    }
    while(1)
    {
      Serial_putc(MD_Num, 0x01);
      delay(5); 
      if(nspeed >= 0.1)
      {
        Serial_putc(MD_Num, 0x80);
        delay(5);
      }
      if(angle_A >= 0)
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
      nh.spinOnce();
    }
  }
  else
  {
    while(1)
    {
      Serial_putc(MD_Num, 0x01);
      delay(5); 
      if(nspeed >= 1.5)
      {
        Serial_putc(MD_Num, 0x80);
        delay(5);
      }
      if(angle_A >= 340)
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
      nh.spinOnce();
    }
    while(1)
    {
      Serial_putc(MD_Num, 0x01);
      delay(5);
      if(nspeed >= 0.1)
      {
        Serial_putc(MD_Num, 0x80);
        delay(5);
      }
      if(angle_A <= 90)
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
      nh.spinOnce();
    }
  }
  while(1)
  {
    Serial_putc(MD_Num_R, 0x08);
    delay(5);
    if(nspeed_R >= 1.5)
    {
      Serial_putc(MD_Num_R, 0x00);
      delay(5);
    }
    if(angle_R >= -230)//-113
    {
      Serial_putc(MD_Num_R, 0x80);
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
    nh.spinOnce();
  }
}

void Receive_Setup_1(void)//投射後に受け渡しアームを受け渡し位置まで戻す
{
  while(1)
  {
    Serial_putc(MD_Num_R, 0x04);
    delay(5);
    if(nspeed_R >= 1.5)
    {
      Serial_putc(MD_Num_R, 0x00);
      delay(5);
    }
    if(angle_R >= -135)
    {
      Serial_putc(MD_Num_R, 0x80);
      delay(5);
      break;
    }
    if(flag_K == 1)
    {
      nh.loginfo("OK");
      break;
    }
    nh.spinOnce();
  }
  while(1)
  {
    Serial_putc(MD_Num_R, 0x01);
    delay(5);
    if(nspeed_R >= 0.3)
    {
      Serial_putc(MD_Num_R, 0x80);
      delay(5);
    }
    if(angle_R >= -115)
    {
      Serial_putc(MD_Num_R, 0x80);
      delay(5);
      break;
    }
    if(flag_K == 1)
    {
      nh.loginfo("OK");
      break;
    }
    nh.spinOnce();
  }
}

void Receive_Setup(void)//最初に受け渡しアームを受け渡し位置まで動かす
{
  while(1)
  {
    nh.spinOnce();
    Serial_putc(MD_Num_R, 0x83);
    delay(5);
    if(nspeed_R <= -1.5)
    {
      Serial_putc(MD_Num_R, 0x00);
      delay(5);
    }
    if(angle_R <= -90)
    {
      Serial_putc(MD_Num_R, 0x80);
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
    Serial_putc(MD_Num_R, 0x81);
    delay(5);
    if(nspeed_R <= -0.3)
    {
      Serial_putc(MD_Num_R, 0x80);
      delay(5);
    }
    if(angle_R <= -115)
    {
      Serial_putc(MD_Num_R, 0x80);
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

