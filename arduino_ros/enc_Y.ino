//pinの割り込み処理
void rotary_changedPin_Y()
{
  int now_a;
  int now_b;
  now_a = digitalRead(pin_YA);
  now_b = digitalRead(pin_YB);
  if( parse_Y == 0 )
  {
    if(( now_a == 1 ) && ( now_b == 0 )) // reverse=CCW
    {
      rot_count_Y+=0.25;
      direction_Y = 0; //CCW
      parse_Y = 3;
      return;
    } else if (( now_a == 0 ) && ( now_b == 1)) //foward = CW
    {
        rot_count_Y-=0.25;
        direction_Y = 1;    //CW
        parse_Y = 1;//1
        return;
    } else {
         // fatal error
    }
  }
  if( parse_Y == 1 )
  {
    if(( now_a == 0 ) && (now_b == 0 )) // reverse = CCW
    {
      rot_count_Y+=0.25;
      direction_Y= 0;    //CCW
      parse_Y = 0;
      return;
    } else if (( now_a == 1 ) && ( now_b == 1)) //foward = CW
    {
        rot_count_Y-=0.25;
        direction_Y = 1;    // CW
        parse_Y = 2;
        return;
    } else {
       // fatal error
    }
  }
 /////////////////////////////////////////////////////////////////
  if( parse_Y == 2 )
  {
    if (( now_a == 1 ) && ( now_b == 0)) //foward = CW
    {
        rot_count_Y-=0.25;
        direction_Y = 0;    // CW
        parse_Y = 3;
        return;
    } else if (( now_a == 0 ) && ( now_b == 1)) //foward = CW
    {
        rot_count_Y+=0.25;
        direction_Y = 1;    //CW
        parse_Y = 1;//1
        return;
    } else {
         // fatal error
    }
  }
  ////////////////////////////////////////////////////////////////////
  if( parse_Y == 3 )
  {
    if(( now_a == 1 ) && (now_b == 1 )) // reverse = CCW
    {
      rot_count_Y+=0.25;
      direction_Y= 0;    //CCW
      parse_Y = 2;
      return;
    } else if (( now_a == 0 ) && ( now_b == 0)) //foward = CW
    {
        rot_count_Y-=0.25;
        direction_Y = 1;    // CW
        parse_Y = 0;
        return;
    } else
    {
       // fatal error
    }
  }
}
