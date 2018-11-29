/* 指定したID(id)を持つデバイスに1byteのデータ(data)を送信(受信割り込みを有効にします) */
void Serial_putc(unsigned char id, unsigned char data)
{
    noInterrupts();
    Serial2.write(SERIAL_FRAMEDELIMITER);
    Serial2.write(id);
    switch(data)
    {
    case SERIAL_FRAMEDELIMITER:
        Serial2.write(SERIAL_ESCAPECHAR);
        Serial2.write(0xff - SERIAL_FRAMEDELIMITER);
        break;
    case SERIAL_ESCAPECHAR:
        Serial2.write(SERIAL_ESCAPECHAR);
        Serial2.write(0xff - SERIAL_ESCAPECHAR);
        break;
    default:
        Serial2.write(data);
        break;
    }
    interrupts();
    return;
}



