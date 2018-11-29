//pinの割り込み処理
void rotary_changedPin_X()
{
  int now_a;
  int now_b;
  now_a = digitalRead(pin_XA);
  now_b = digitalRead(pin_XB);
  if( parse_X == 0 )
  {
    if(( now_a == 1 ) && ( now_b == 0 )) // reverse=CCW
    {
      rot_count_X+=0.25;
      direction_X = 0; //CCW
      parse_X = 3;
      return;
    } else if (( now_a == 0 ) && ( now_b == 1)) //foward = CW
    {
        rot_count_X-=0.25;
        direction_X = 1;    //CW
        parse_X = 1;//1
        return;
    } else {
         // fatal error
    }
  }
  if( parse_X == 1 )
  {
    if(( now_a == 0 ) && (now_b == 0 )) // reverse = CCW
    {
      rot_count_X+=0.25;
      direction_X= 0;    //CCW
      parse_X = 0;
      return;
    } else if (( now_a == 1 ) && ( now_b == 1)) //foward = CW
    {
        rot_count_X-=0.25;
        direction_X = 1;    // CW
        parse_X = 2;
        return;
    } else {
       // fatal error
    }
  }
 /////////////////////////////////////////////////////////////////
  if( parse_X == 2 )
  {
    if (( now_a == 1 ) && ( now_b == 0)) //foward = CW
    {
        rot_count_X-=0.25;
        direction_X = 0;    // CW
        parse_X = 3;
        return;
    } else if (( now_a == 0 ) && ( now_b == 1)) //foward = CW
    {
        rot_count_X+=0.25;
        direction_X = 1;    //CW
        parse_X = 1;//1
        return;
    } else {
         // fatal error
    }
  }
  ////////////////////////////////////////////////////////////////////
  if( parse_X == 3 )
  {
    if(( now_a == 1 ) && (now_b == 1 )) // reverse = CCW
    {
      rot_count_X+=0.25;
      direction_X= 0;    //CCW
      parse_X = 2;
      return;
    } else if (( now_a == 0 ) && ( now_b == 0)) //foward = CW
    {
        rot_count_X-=0.25;
        direction_X = 1;    // CW
        parse_X = 0;
        return;
    } else
    {
       // fatal error
    }
  }
}
