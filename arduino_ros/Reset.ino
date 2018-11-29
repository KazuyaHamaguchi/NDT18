void reset()
{
  if(!first)
  {
    Serial_putc(FR_N, 0x00);
    Serial_putc(FL_N, 0x00);
    Serial_putc(RR_N, 0x00);
    Serial_putc(RL_N, 0x00);
    rot_count_X = 0.0;
    pre_pulse_X = 0.0, cur_pulse_X = 0.0;
    sokudo_X = 0.0;
    direction_X = 0;
    parse_X;
    rot_count_Y = 0.0;
    pre_pulse_Y = 0.0, cur_pulse_Y = 0.0;
    sokudo_Y = 0.0;
    direction_Y = 0;
    parse_Y;
    enc_msg.distance_X = 0;
    enc_msg.speed_X = 0;
    enc_msg.distance_Y = 0;
    enc_msg.speed_Y = 0;
    pub_enc.publish(&enc_msg);
    nh.loginfo("Reset complete");
    first = true;
  }
}

