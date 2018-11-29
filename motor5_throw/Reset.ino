void Res(void)//リセット
{
  if(flag_K == 0)
  {
    flag_K = 1;
    digitalWrite(airPin_R, LOW);
    digitalWrite(airPin, LOW);
    rot_count = 0;
    angle = 0;
    direction = 0;
    parse = 0;
    count = 0;
    count_A = 0;
    nspeed = 0;
    Setspeed = 0;
    Setangle = 0;
    r = 0.45;
    angle_A = 0; 
    angle_B = 0;
    rot_count_R = 0;
    angle_R = 0;
    rot_count_rpm = 0;
    direction_R = 0;
    parse_R = 0;
    count_R = 0;
    count_AR = 0;
    nspeed_R = 0;
    diff[2] = 0;
    integral = 0;
    aw_bc = 0;
    aw_bc_old = 0;
    data = 0;
    cir = 0;
    buttonState = 0;
    Output = 0;
    TZ3_deg = 25.67;
  }
  else;
}

void reset()
{
  if(!first)
  {
    flag_K = 1;
    digitalWrite(airPin_R, LOW);
    digitalWrite(airPin, LOW);
    rot_count = 0;
    angle = 0;
    direction = 0;
    parse = 0;
    count = 0;
    count_A = 0;
    nspeed = 0;
    Setspeed = 0;
    Setangle = 0;
    r = 0.45;
    angle_A = 0; 
    angle_B = 0;
    rot_count_R = 0;
    angle_R = 0;
    rot_count_rpm = 0;
    direction_R = 0;
    parse_R = 0;
    count_R = 0;
    count_AR = 0;
    nspeed_R = 0;
    diff[2] = 0;
    integral = 0;
    aw_bc = 0;
    aw_bc_old = 0;
    data = 0;
    cir = 0;
    buttonState = 0;
    Output = 0;
    nh.loginfo("Throw_Ar Reset complete");
    first = true;
  }
}
