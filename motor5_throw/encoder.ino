/*pinの割り込み処理*/
void rotary_changedPin_R(void)
{
  if(rot_count_R == 720 || rot_count_R == -720)
  {
    rot_count_R = 0;
  }
  angle_R = rot_count_R / 2;
  int now_a;
  int now_b;
  now_a = digitalRead(pin_A);
  now_b = digitalRead(pin_B);
  if ( parse_R == 0 )
  {
    if (( now_a == 1 ) && ( now_b == 0 )) // reverse=CCW
    {
      rot_count_R += 0.5;
      rot_count_rpm += 0.5;
      direction_R = 0; //CCW
      parse_R = 3;
      return;
    } else if (( now_a == 0 ) && ( now_b == 1)) //foward = CW
    {
      rot_count_R -= 0.5;
      rot_count_rpm -= 0.5;
      direction_R = 1;    //CW
      parse_R = 1;//1
      return;
    } else {
      // fatal error
    }
  }
  if ( parse_R == 1 )
  {
    if (( now_a == 0 ) && (now_b == 0 )) // reverse = CCW
    {
      rot_count_R += 0.5;
      rot_count_rpm += 0.5;
      direction_R = 0;   //CCW
      parse_R = 0;
      return;
    } else if (( now_a == 1 ) && ( now_b == 1)) //foward = CW
    {
      rot_count_R -= 0.5;
      rot_count_rpm -= 0.5;
      direction_R = 1;    // CW
      parse_R = 2;
      return;
    } else {
      // fatal error
    }
  }
  /////////////////////////////////////////////////////////////////
  if ( parse_R == 2 )
  {
    if (( now_a == 1 ) && ( now_b == 0)) //foward = CW
    {
      rot_count_R -= 0.5;
      rot_count_rpm -= 0.5;
      direction_R = 0;    // CW
      parse_R = 3;
      return;
    } else if (( now_a == 0 ) && ( now_b == 1)) //foward = CW
    {
      rot_count_R += 0.5;
      rot_count_rpm += 0.5;
      direction_R = 1;    //CW
      parse_R = 1;//1
      return;
    } else {
      // fatal error
    }
  }
  ////////////////////////////////////////////////////////////////////
  if ( parse_R == 3 )
  {
    if (( now_a == 1 ) && (now_b == 1 )) // reverse = CCW
    {
      rot_count_R += 0.5;
      rot_count_rpm += 0.5;
      direction_R = 0;   //CCW
      parse_R = 2;
      return;
    } else if (( now_a == 0 ) && ( now_b == 0)) //foward = CW
    {
      rot_count_R -= 0.5;
      rot_count_rpm -= 0.5;
      direction_R = 1;    // CW
      parse_R = 0;
      return;
    } else {
      // fatal error
    }
  }
}

/*pinの割り込み処理*/
void rotary_changedPin(void)
{
  if(angle == 720 || angle == -720)
  {
    angle = 0;
  }
  angle_A = angle / 2;
  int now_a;
  int now_b;
  now_a = digitalRead(pinA);
  now_b = digitalRead(pinB);
  if ( parse == 0 )
  {
    if (( now_a == 1 ) && ( now_b == 0 )) // reverse=CCW
    {
      rot_count += 0.125;
      angle += 0.125;
      direction = 0; //CCW
      parse = 3;
      return;
    } else if (( now_a == 0 ) && ( now_b == 1)) //foward = CW
    {
      rot_count -= 0.125;
      angle -= 0.125;
      direction = 1;    //CW
      parse = 1;//1
      return;
    } else {
      // fatal error
    }
  }
  if ( parse == 1 )
  {
    if (( now_a == 0 ) && (now_b == 0 )) // reverse = CCW
    {
      rot_count += 0.125;
      angle += 0.125;
      direction = 0;   //CCW
      parse = 0;
      return;
    } else if (( now_a == 1 ) && ( now_b == 1)) //foward = CW
    {
      rot_count -= 0.125;
      angle -= 0.125;
      direction = 1;    // CW
      parse = 2;
      return;
    } else {
      // fatal error
    }
  }
  /////////////////////////////////////////////////////////////////
  if ( parse == 2 )
  {
    if (( now_a == 1 ) && ( now_b == 0)) //foward = CW
    {
      rot_count -= 0.125;
      angle -= 0.125;
      direction = 0;    // CW
      parse = 3;
      return;
    } else if (( now_a == 0 ) && ( now_b == 1)) //foward = CW
    {
      rot_count += 0.125;
      angle += 0.125;
      direction = 1;    //CW
      parse = 1;//1
      return;
    } else {
      // fatal error
    }
  }
  ////////////////////////////////////////////////////////////////////
  if ( parse == 3 )
  {
    if (( now_a == 1 ) && (now_b == 1 )) // reverse = CCW
    {
      rot_count += 0.125;
      angle += 0.125;
      direction = 0;   //CCW
      parse = 2;
      return;
    } else if (( now_a == 0 ) && ( now_b == 0)) //foward = CW
    {
      rot_count -= 0.125;
      angle -= 0.125;
      direction = 1;    // CW
      parse = 0;
      return;
    } else {
      // fatal error
    }
  }
}
