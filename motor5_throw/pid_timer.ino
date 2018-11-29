/* 受信割り込み */
void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t mSec) {
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
  uint32_t rc = (VARIANT_MCK / 2 / 1000) * mSec;
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}

float pid_smart(float sensor_val, float target_val, float KP, float KI, float KD, float KB, int AW) {
  float p, i, d, _ki, _kb;

  diff[0] = diff[1];
  diff[1] = target_val - sensor_val; //偏差を取得
  _ki = 0.5 * KI * DELTA_T;
  integral += (diff[1] + diff[0]) * _ki;  //演算速度 : 1msec
  if(AW == 1 && _ki != 0)
  {
    _kb = 0.5 * KB * DELTA_T;
  }
  else
  {
    _kb = 0;
  }
  //Serial.println(diff[1]);

  p = KP * diff[1];                                   // 比例
  i = integral;                                  // 積分
  d = KD * (diff[1] - diff[0]) / DELTA_T;             // 微分

  double u = p + i + d;
  double aw_bc = constrain(u, 0x00, 0x2F) - u;
  i += _kb * (aw_bc + aw_bc_old);
  aw_bc_old = aw_bc;

  Output = p + i + d;
  Output = constrain(Output, 0x00, 0x2F);             // モーターの出力範囲 : 0-2F
  //Serial.println(Output, HEX);
}
