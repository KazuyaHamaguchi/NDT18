/*10進数を16進数に変換*/
unsigned int HEX_conversion(int DEC_sokudo)
{
  int sokudo = DEC_sokudo, amari;
  double sokudo_a, sokudo_b;
  if(sokudo >= 0 && sokudo <= 100)
  {
    sokudo_a = sokudo * 127.0 / 100.0;
  }
  else if(sokudo < 0 && sokudo >= -100)
  {
    sokudo_a = sokudo * (255.0 - 129.0) / -100.0 + 129.0;
  }
  else if(sokudo == 8080)
  {
    return 0x80;
  }
  else
  {
    return 0x00;
  }
  sokudo = sokudo_a;
  sokudo_b = sokudo;
  sokudo_a = sokudo_a - sokudo_b;
  if(sokudo_a >= 0.5000)
  {
    sokudo++;
  }
  else;
  amari = sokudo % 16;
  sokudo = sokudo / 16;
  switch(amari)
  {
    case 0:
      amari = 0x00;
      break;

    case 1:
      amari = 0x01;
      break;

    case 2:
      amari = 0x02;
      break;

    case 3:
      amari = 0x03;
      break;

    case 4:
      amari = 0x04;
      break;

    case 5:
      amari = 0x05;
      break;

    case 6:
      amari = 0x06;
      break;

    case 7:
      amari = 0x07;
      break;

    case 8:
      amari = 0x08;
      break;
          
    case 9:
      amari = 0x09;
      break;
          
    case 10:
      amari = 0x0a;
      break;

    case 11:
      amari = 0x0b;
      break;
      
    case 12:
      amari = 0x0c;
      break;

    case 13:
      amari = 0x0d;
      break;

    case 14:
      amari = 0x0e;
      break;

    case 15:
      amari = 0x0f;
      break;

    default:
      break;
  }
  switch(sokudo)
  {
    case 0:
      sokudo = 0x00;
      break;
      
    case 1:
      sokudo = 0x10;
      break;

    case 2:
      sokudo = 0x20;
      break;
      
    case 3:
      sokudo = 0x30;
      break;
      
    case 4:
      sokudo = 0x40;
      break;

    case 5:
      sokudo = 0x50;
      break;

    case 6:
      sokudo = 0x60;
      break;

    case 7:
      sokudo = 0x70; 
      break;

    case 8:
      sokudo = 0x80;
      break;

    case 9:
      sokudo = 0x90;
      break;

    case 10:
      sokudo = 0xa0;
      break;
      
    case 11:
      sokudo = 0xb0;
      break;

    case 12:
      sokudo = 0xc0;
      break;

    case 13:
      sokudo = 0xd0;
      break;

    case 14:
      sokudo = 0xe0; 
      break;

    case 15:
      sokudo = 0xf0;
      break;
      
    default:
      break;      
  }
  sokudo = sokudo + amari;
  return sokudo;
}
