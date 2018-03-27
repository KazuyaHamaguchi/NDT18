/*******************************************************************
* Strawberry Linux社の「MPU-9250」からI2Cでデータを取得するクラス（C++）
* https://strawberry-linux.com/catalog/items?code=12250
*
* 2018-01-20 Kazuya Hamaguchi
*
* 2017-11-27 作成開始
* 2018-01-09 仮完成
* 2018-01-20 キャリブレーション追加
* 2018-02-21 仮完成2
* 2018-02-23 Gyro zのみでクオータニオン 実装
*
* 参考にしたサイト
* https://github.com/simondlevy/RPi_MPU9250
* https://qiita.com/boyaki_machine/items/915f7730c737f2a5cc79
* https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
*******************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <mpu9250/distance.h>

#include <pigpiod_if2.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

int pi = pigpio_start(0, 0);

enum mpu9250_gyro_range
{
    GYRO_RANGE_250DPS,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS
};

enum mpu9250_accel_range
{
    ACCEL_RANGE_2G,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G
};

enum mpu9250_dlpf_bandwidth
{
    DLPF_BANDWIDTH_184HZ,
    DLPF_BANDWIDTH_92HZ,
    DLPF_BANDWIDTH_41HZ,
    DLPF_BANDWIDTH_20HZ,
    DLPF_BANDWIDTH_10HZ,
    DLPF_BANDWIDTH_5HZ
};

class MPU9250
{
public:
    MPU9250(uint8_t i2c_address);
    int begin(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange);
    int setFilt(mpu9250_dlpf_bandwidth bandwidth, uint8_t SRD);
    int u2s(unsigned unsigneddata);
    void getAccel(float* ax, float* ay, float* az);
    void getGyro(float* gx, float* gy, float* gz);
    void getMag(float* hx, float* hy, float* hz);
    void getTemp(float *t);

    void getAccelCounts(int16_t* ax, int16_t* ay, int16_t* az);
    void getGyroCounts(int16_t* gx, int16_t* gy, int16_t* gz);
    void getMagCounts(int16_t* hx, int16_t* hy, int16_t* hz);
    void getTempCounts(int16_t* t);

    float calibAccel();
    float calibGyro();
    //void calibMag();


private:
    uint8_t _i2c_address;
    uint8_t _i2c_handle;
    uint8_t _AK8963_handle;
    float _accelScale;
    float _gyroScale;
    float _magScaleX, _magScaleY, _magScaleZ;
    float _offsetAccelX, _offsetAccelY, _offsetAccelZ;
    float _offsetGyroX, _offsetGyroY, _offsetGyroZ;
    static const float _tempScale = 333.87f;
    static const float _tempOffset = 21.0f;

    // 定数
    static const float G = 9.80665f;

    // MPU9250レジスタ
    static const uint8_t ACCEL_OUT = 0x3B;
    static const uint8_t GYRO_OUT = 0x43;
    static const uint8_t TEMP_OUT = 0x41;

    static const uint8_t ACCEL_CONFIG = 0x1C;
    static const uint8_t ACCEL_FS_SEL_2G = 0x00;
    static const uint8_t ACCEL_FS_SEL_4G = 0x08;
    static const uint8_t ACCEL_FS_SEL_8G = 0x10;
    static const uint8_t ACCEL_FS_SEL_16G = 0x18;

    static const uint8_t GYRO_CONFIG = 0x1B;
    static const uint8_t GYRO_FS_SEL_250DPS = 0x00;
    static const uint8_t GYRO_FS_SEL_500DPS = 0x08;
    static const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
    static const uint8_t GYRO_FS_SEL_2000DPS = 0x18;

    static const uint8_t ACCEL_CONFIG2 = 0x1D;
    static const uint8_t ACCEL_DLPF_184 = 0x01;
    static const uint8_t ACCEL_DLPF_92 = 0x02;
    static const uint8_t ACCEL_DLPF_41 = 0x03;
    static const uint8_t ACCEL_DLPF_20 = 0x04;
    static const uint8_t ACCEL_DLPF_10 = 0x05;
    static const uint8_t ACCEL_DLPF_5 = 0x06;

    static const uint8_t CONFIG = 0x1A;
    static const uint8_t GYRO_DLPF_184 = 0x01;
    static const uint8_t GYRO_DLPF_92 = 0x02;
    static const uint8_t GYRO_DLPF_41 = 0x03;
    static const uint8_t GYRO_DLPF_20 = 0x04;
    static const uint8_t GYRO_DLPF_10 = 0x05;
    static const uint8_t GYRO_DLPF_5 = 0x06;

    static const uint8_t SMPDIV = 0x19;

    static const uint8_t INT_PIN_CFG = 0x37;
    static const uint8_t INT_ENABLE = 0x38;
    static const uint8_t INT_RAW_RDY_EN = 0x01;

    static const uint8_t PWR_MGMNT_1 = 0x6B;
    static const uint8_t PWR_RESET = 0x80;
    static const uint8_t CLOCK_SEL_PLL = 0x01;

    static const uint8_t PWR_MGMNT_2 = 0x6C;
    static const uint8_t SEN_ENABLE = 0x00;

    static const uint8_t I2C_MST_CLK = 0x0D;
    static const uint8_t I2C_MST_CTRL = 0x24;

    static const uint8_t I2C_SLV1_MASK = 0x02;

    static const uint8_t WHO_AM_I = 0x75;

    // AK8963(地磁気)レジスタ
    static const uint8_t AK8963_I2C_ADDR = 0x0C;

    static const uint8_t AK8963_HXL = 0x03;

    static const uint8_t AK8963_CNTL1 = 0x0A;
    static const uint8_t AK8963_PWR_DOWN = 0x00;
    static const uint8_t AK8963_CNT_MEAS2 = 0x16;
    static const uint8_t AK8963_SELF_TEST = 0x18;
    static const uint8_t AK8963_FUSE_ROM = 0x0F;

    static const uint8_t AK8963_CNTL2 = 0x0B;
    static const uint8_t AK8963_RESET = 0x01;

    static const uint8_t AK8963_ST1 = 0x02;

    static const uint8_t AK8963_ASTC = 0x0C;
    static const uint8_t AK8963_SELF_ON = 0x40;
    static const uint8_t AK9063_SELF_OFF = 0x00;

    static const uint8_t AK8963_ASA = 0x10;

    static const uint8_t AK8963_WHO_AM_I = 0x00;

    // 変換行列
    /* 加速度センサの軸とジャイロセンサの軸を変換して地磁気センサの軸に一致させる */
    static const int16_t tX[3];
    static const int16_t tY[3];
    static const int16_t tZ[3];

    bool writeRegister(uint8_t subAddress, uint8_t data);
    void readRegisters(uint8_t subAddress, uint8_t count, char* buf);
    bool writeAK8963Register(uint8_t subAddress, uint8_t data);
    void readAK8953Registers(uint8_t subAddress, uint8_t count, char* buf);
    uint8_t whoAmI();
    uint8_t whoAmIAK8963();
};


/********************************************************************************************************************/


MPU9250 IMU(0x68);

float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
float lastgz, yaw;
int beginStatus, setFilter;

// ローパスフィルターの係数(これは環境によって要調整。1に近づけるほど平滑化の度合いが大きくなる)
float filterCoefficient = 0.6;
float lowpassValue = 0;
float highpassValue = 0;

// 時間差分
float timeSpan = 0.01;
// ひとつ前の加速度
float oldAccel = 0;
//　加速度から算出した速度
float speed = 0;
// ひとつ前の速度
float oldSpeed = 0;
// 速度から算出した変位
float difference = 0;

float imu_distance = 0;


//static void setup();
//static void loop();
float toDifference(float accel);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "MPU9250");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);
    ros::Publisher imu_raw_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 10);
    ros::Publisher mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
    ros::Publisher dis_pub_ = nh.advertise<mpu9250::distance>("distance", 10);
    sensor_msgs::Imu imu_msg_;
    sensor_msgs::MagneticField mag_msg_;
    mpu9250::distance dis_msg_;

    imu_msg_.header.frame_id = "imu";

    beginStatus = IMU.begin(ACCEL_RANGE_2G, GYRO_RANGE_1000DPS);
    setFilter = IMU.setFilt(DLPF_BANDWIDTH_41HZ, 0);
    //setup();

    while(ros::ok())
    {
        if(beginStatus < 0 || setFilter < 0)
        {
            fprintf(stderr, "IMU initialization unsuccessful\n");
            fprintf(stderr, "Check IMU wiring or try cycling power\n");
            time_sleep(2);
        }
        else
        {
            IMU.getAccel(&ax, &ay, &az);

            IMU.getGyro(&gx, &gy, &gz);

            IMU.getMag(&hx, &hy, &hz);

            IMU.getTemp(&t);

            imu_distance = toDifference(ax);
            dis_msg_.disy = imu_distance;
            //printf("%f\n", imu_distance);
            dis_pub_.publish(dis_msg_);

            float cr2 = cos(0*0.5);
            float cp2 = cos(0*0.5);
            float cy2 = cos(gz*0.5);
            float sr2 = sin(0*0.5);
            float sp2 = sin(0*0.5);
            float sy2 = sin(gz*0.5);

            imu_msg_.orientation.w = cr2*cp2*cy2 + sr2*sp2*sy2;
            imu_msg_.orientation.x = sr2*cp2*cy2 - cr2*sp2*sy2;
            imu_msg_.orientation.y = cr2*sp2*cy2 + sr2*cp2*sy2;
            imu_msg_.orientation.z = cr2*cp2*sy2 - sr2*sp2*cy2;





        //loop();

            ros::Time current_time = ros::Time::now();
            imu_msg_.header.stamp = current_time;
            imu_msg_.linear_acceleration.x = ax;
            imu_msg_.linear_acceleration.y = ay;
            imu_msg_.linear_acceleration.z = az;
            imu_msg_.angular_velocity.x = gx;
            imu_msg_.angular_velocity.y = gy;
            imu_msg_.angular_velocity.z = gz;
            imu_raw_pub_.publish(imu_msg_);

            mag_msg_.header.stamp = current_time;
            mag_msg_.magnetic_field.x = hx;
            mag_msg_.magnetic_field.y = hy;
            mag_msg_.magnetic_field.z = hz;
            mag_pub_.publish(mag_msg_);

            ros::spinOnce();
            loop_rate.sleep();
            //lastyaw = yaw;
        }
    }
}

/****************************************************/
float toDifference(float accel)
{
    // ローパスフィルター(現在の値 = 係数 * ひとつ前の値 ＋ (1 - 係数) * センサの値)
    lowpassValue = lowpassValue * filterCoefficient + accel * (1 - filterCoefficient);
    // ハイパスフィルター(センサの値 - ローパスフィルターの値)
    //highpassValue = accel - lowpassValue;

    // 速度計算(加速度を台形積分する)
    //speed += ((oldAccel + accel) * timeSpan) / 2;
    //speed = ((highpassValue + oldAccel) * timeSpan) / 2 + speed;
    speed = (((oldAccel + lowpassValue) * timeSpan) / 2) + speed;
    oldAccel = lowpassValue;
    //oldAccel = highpassValue;
    // 変位計算(速度を台形積分する)
    //difference = ((speed + oldSpeed) * timeSpan) / 2 + difference;
    //oldSpeed = speed;

    return speed;
}
/****************************************************/

/*static void setup()
{
    beginStatus = IMU.begin(ACCEL_RANGE_2G, GYRO_RANGE_1000DPS);
    setFilter = IMU.setFilt(DLPF_BANDWIDTH_41HZ, 0);
}

static void loop()
{
    if(beginStatus < 0 || setFilter < 0)
    {
        time_sleep(1);
        fprintf(stderr, "IMU initialization unsuccessful\n");
        fprintf(stderr, "Check IMU wiring or try cycling power\n");
        time_sleep(5);
    }
    else
    {
        IMU.getAccel(&ax, &ay, &az);

        IMU.getGyro(&gx, &gy, &gz);

        IMU.getMag(&hx, &hy, &hz);

        IMU.getTemp(&t);

        ros::Time current_time = ros::Time::now();

        imu_msg_.header.stamp = current_time;
        imu_msg_.linear_acceleration.x = ax;
        imu_msg_.linear_acceleration.y = ay;
        imu_msg_.linear_acceleration.z = az;
        imu_msg_.angular_velocity.x = gx;
        imu_msg_.angular_velocity.y = gy;
        imu_msg_.angular_velocity.z = gz;
        imu_raw_pub_.publish(imu_msg_);

        printData();
    }
}*/



/**********************************************************************************************************************/


/* MPU9250オブジェクト，I2Cアドレスを入力 */
MPU9250::MPU9250(uint8_t i2c_address)
{
    _i2c_address = i2c_address; // I2Cアドレス
}

/* I2C通信を開始しMPU-9250をセットアップする */
int MPU9250::begin(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange)
{
    float hx, hy, hz;
    char buff[3];
    char data[7];

    // I2Cバスを開始する
    _i2c_handle = i2c_open(pi, 1, _i2c_address, 0);
    _AK8963_handle = i2c_open(pi, 1, AK8963_I2C_ADDR, 0);

    if(_i2c_handle < 0)
    {
        printf("1\n");
        return -1;
    }
    if(_AK8963_handle < 0)
    {
        printf("2\n");
        return -1;
    }

        // MPU9250のレジスタをリセットする
        writeRegister(PWR_MGMNT_1, PWR_RESET);
        // AK8963のレジスタをリセットする
        writeAK8963Register(AK8963_CNTL2, AK8963_RESET);

    time_sleep(0.1);

        // レジスタをセンシング可能な状態にする
        writeRegister(PWR_MGMNT_1, SEN_ENABLE);
        // I2Cで磁気センサ機能(AK8963)へアクセスできるようにする(BYPASS_EN=1)
        writeRegister(INT_PIN_CFG, I2C_SLV1_MASK);

    // ジャイロのクロックソースを選択
    /*if( !writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL) )
    {
        return -1;
    }*/

    // WHO AM Iバイトをチェックする，期待値は0x71(10進数で113)
    if( whoAmI() != 0x71)
    {
        printf("3\n");
        return -1;
    }

    // 加速度センサとジャイロセンサを有効にする
    if( !writeRegister(PWR_MGMNT_2, SEN_ENABLE) )
    {
        printf("4\n");
        return -1;
    }

    // 加速度センサとジャイロセンサの測定レンジを設定する
    // 広レンジでは測定粒度が荒くなる
    switch(accelRange)
    {
        case ACCEL_RANGE_2G:
            // 加速度センサの測定レンジを±2Gに設定する
            if( !writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_2G) )
            {
                printf("5\n");
                return -1;
            }
            _accelScale = 2.0f/32767.5f; // 加速度センサの測定範囲を2Gに設定
            break;

        case ACCEL_RANGE_4G:
            // 加速度センサの測定レンジを±4Gに設定する
            if( !writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_4G) )
            {
                printf("5\n");
                return -1;
            }
            _accelScale = 4.0f/32767.5f; // 加速度センサの測定範囲を4Gに設定
            break;

        case ACCEL_RANGE_8G:
            // 加速度センサの測定レンジを±8Gに設定する
            if( !writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_8G) )
            {
                printf("5\n");
                return -1;
            }
            _accelScale = 8.0f/32767.5f; // 加速度センサの測定範囲を8Gに設定
            break;

        case ACCEL_RANGE_16G:
            // 加速度センサの測定レンジを±16Gに設定する
            if( !writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_16G) )
            {
                printf("5\n");
                return -1;
            }
            _accelScale = 16.0f/32767.5f; // 加速度センサの測定範囲を16Gに設定
            break;
    }

    calibAccel();

    switch(gyroRange)
    {
        case GYRO_RANGE_250DPS:
            // ジャイロセンサの測定レンジを250DPSに設定する
            if( !writeRegister(GYRO_CONFIG, GYRO_FS_SEL_250DPS) )
            {
                printf("6\n");
                return -1;
            }
            _gyroScale = 250.0f/32767.5f; // ジャイロセンサの測定範囲を250DPSに設定
            break;

        case GYRO_RANGE_500DPS:
            // ジャイロセンサの測定レンジを500DPSに設定する
            if( !writeRegister(GYRO_CONFIG, GYRO_FS_SEL_500DPS) )
            {
                printf("6\n");
                return -1;
            }
            _gyroScale = 500.0f/32767.5f; // ジャイロセンサの測定範囲を500DPSに設定
            break;

        case GYRO_RANGE_1000DPS:
            // ジャイロセンサの測定レンジを1000DPSに設定する
            if( !writeRegister(GYRO_CONFIG, GYRO_FS_SEL_1000DPS) )
            {
                printf("6\n");
                return -1;
            }
            _gyroScale = 1000.0f/32767.5f; // ジャイロセンサの測定範囲を1000DPSに設定
            break;

        case GYRO_RANGE_2000DPS:
            // ジャイロセンサの測定レンジを2000DPSに設定する
            if( !writeRegister(GYRO_CONFIG, GYRO_FS_SEL_2000DPS) )
            {
                printf("6\n");
                return -1;
            }
            _gyroScale = 2000.0f/32767.5f; // ジャイロセンサの測定範囲を2000DPSに設定
            break;
    }

    calibGyro();

        // I2Cバス速度を400kHzに設定する
        if( !writeRegister(I2C_MST_CTRL, I2C_MST_CLK) )
        {
            printf("7\n");
            return -1;
        }

        // AK8963のWHO AM Iバイトをチェックする，期待値は0x48(10進数で72)
        if( whoAmIAK8963() != 72 )
        {
            printf("8\n");
            return -1;
        }


    /* 地磁気センサーのキャリブレーションを取得する */

    // AK8963をパワーダウンモードに設定
    writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

    // ヒューズROMアクセスモードに設定
    writeAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM);

    // AK8963のASAレジスタを読み取り，地磁気センサのスケールファクタを計算する
    readAK8953Registers(AK8963_ASA, sizeof(buff), buff);
    time_sleep(0.0001); // これらのレジスタがいっぱいになるまでに時間がかかる

    // 感度調整式(AK8953データシート32ページ) ← ((float)buff[0]) - 128.0f * 0.5f / 128.0f + 1.0f
    // uTへの変換(AK8963データシート29ページ) ← 4912.0f / 32760.0
    _magScaleX = ((((((float)buff[0]) - 128.0f) * 0.5f) / 128.0f) + 1.0f) * (4912.0f / 32760.0f);   // buff[0] = ASAX = 180.0 (実測値)
    _magScaleY = ((((((float)buff[1]) - 128.0f) * 0.5f) / 128.0f) + 1.0f) * (4912.0f / 32760.0f);   // buff[1] = ASAY = 180.0 (実測値)
    _magScaleZ = ((((((float)buff[2]) - 128.0f) * 0.5f) / 128.0f) + 1.0f) * (4912.0f / 32760.0f);   // buff[2] = ASAZ = 170.0 (実測値)


    /* 地磁気センサのセルフテストを実行する */

    // AK8963をパワーダウンモードに設定
    writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

    // ASTCレジスタのSELFビットに"1"を書き込む
    writeAK8963Register(AK8963_ASTC, AK8963_SELF_ON);

    // セルフテストモードに設定
    writeAK8963Register(AK8963_CNTL1, AK8963_SELF_TEST);

    //ST1レジスタを確認してデータ読み出しが可能か確認する
    readAK8953Registers(AK8963_ST1, sizeof(buff), buff);
    while((buff[0] & 0x01) != 0x01)
    {
        time_sleep(0.01);
        readAK8953Registers(AK8963_ST1, sizeof(buff), buff);
    }

    i2c_read_i2c_block_data(pi, _AK8963_handle, 0x03, data, 6);
    hx = u2s(data[1] << 8 | data[0]);
    hy = u2s(data[3] << 8 | data[2]); //下位bitが先
    hz = u2s(data[5] << 8 | data[4]); //下位bitが先

    // ASTCレジスタのSELFビットに"0"を書き込む
    writeAK8963Register(AK8963_ASTC, AK9063_SELF_OFF);

    // AK8963をパワーダウンモードに設定
    writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

    // セルフテストの判定
    hx = hx * _magScaleX;
    hy = hy * _magScaleY;
    hz = hz * _magScaleZ;

    if(!(-200 <= hx && hx <= 200 || -200 <= hy && hy <= 200 || -3200 <= hz && hz <= -800))
    {
        printf("9\n");
        return -1;
    }

    // AK8963を16ビット分解能，100Hz更新レートに設定
    writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2);

    return 0;
}


/* Low-Pass-Filterと割り込みの設定 */
int MPU9250::setFilt(mpu9250_dlpf_bandwidth bandwidth, uint8_t SRD)
{
    char data[7];

    switch(bandwidth)
    {
        case DLPF_BANDWIDTH_184HZ:
            if( !writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_184) ) // 加速度センサの帯域幅を184Hzに設定
            {
                printf("10\n");
                return -1;
            }
            if( !writeRegister(CONFIG, GYRO_DLPF_184) ) // ジャイロセンサの帯域幅を184Hzに設定
            {
                printf("10\n");
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_92HZ:
            if( !writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_92) ) // 加速度センサの帯域幅を92Hzに設定
            {
                printf("10\n");
                return -1;
            }
            if( !writeRegister(CONFIG, GYRO_DLPF_92) ) // ジャイロセンサの帯域幅を92Hzに設定
            {
                printf("10\n");
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_41HZ:
            if( !writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_41) ) // 加速度センサの帯域幅を41Hzに設定
            {
                printf("10\n");
                return -1;
            }
            if( !writeRegister(CONFIG, GYRO_DLPF_41) ) // ジャイロセンサの帯域幅を41Hzに設定
            {
                printf("10\n");
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_20HZ:
            if( !writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_20) ) // 加速度センサの帯域幅を20Hzに設定
            {
                printf("10\n");
                return -1;
            }
            if( !writeRegister(CONFIG, GYRO_DLPF_20) ) // ジャイロセンサの帯域幅を20Hzに設定
            {
                printf("10\n");
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_10HZ:
            if( !writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_10) ) // 加速度センサの帯域幅を10Hzに設定
            {
                printf("10\n");
                return -1;
            }
            if( !writeRegister(CONFIG, GYRO_DLPF_10) ) // ジャイロセンサの帯域幅を10Hzに設定
            {
                printf("10\n");
                return -1;
            }
            break;

        case DLPF_BANDWIDTH_5HZ:
            if( !writeRegister(ACCEL_CONFIG2, ACCEL_DLPF_5) ) // 加速度センサの帯域幅を5Hzに設定
            {
                printf("10\n");
                return -1;
            }
            if( !writeRegister(CONFIG, GYRO_DLPF_5) ) // ジャイロセンサの帯域幅を5Hzに設定
            {
                printf("10\n");
                return -1;
            }
            break;
    }

    /* サンプルレートデバイダの設定 */
    if( !writeRegister(SMPDIV, SRD) )
    {
        printf("11\n");
        return -1;
    }

    if(SRD > 9)
    {
        // AK8963をパワーダウンモードに設定
        writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);

        // AK8963を16ビット分解能，100Hz更新レートに設定
        writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2);
    }

    if( !writeRegister(INT_ENABLE, INT_RAW_RDY_EN) ) // データ受信準備完了
    {
        printf("12\n");
        return -1;
    }

    // フィルターのセットアップに成功したら0を返す
    return 0;
}


/* 加速度センサデータを取得して値を格納するポインタを取得し，データをカウントとして返す */
void MPU9250::getAccelCounts(int16_t* ax, int16_t* ay, int16_t* az)
{
    char buff[6];
    int16_t axx, ayy, azz;

    readRegisters(ACCEL_OUT, sizeof(buff), buff); // MPU9250からデータを取得する

    *ax = -(u2s((((int16_t)buff[0]) << 8) | buff[1])); // 16ビット値に変換
    *ay = -(u2s((((int16_t)buff[2]) << 8) | buff[3])); //軸変換「-」(センサの向きがx，yそれぞれ負の向きなので)
    *az = u2s((((int16_t)buff[4]) << 8) | buff[5]);
}

float MPU9250::calibAccel()
{
    cout << "Accel calibration start" << endl;
    float sum[3] = {0, 0, 0};
    int16_t accel[3];

    for (int i = 0; i < 1000; i++)
    {
        getAccelCounts(&accel[0], &accel[1], &accel[2]);
        sum[0] += ((float)accel[0]) * _accelScale;
        sum[1] += ((float)accel[1]) * _accelScale;
        sum[2] += ((float)accel[2]) * _accelScale;
    }

    // 平均値をオフセットにする
    _offsetAccelX = -1.0 * sum[0] / 1000;
    _offsetAccelY = -1.0 * sum[1] / 1000;
    _offsetAccelZ = -1.0 * ((sum[2] / 1000) - 1.0);

    printf("offsetAccelX = %6.6f\t", _offsetAccelX);
    printf("offsetAccelY = %6.6f\t", _offsetAccelY);
    printf("offsetAccelZ = %6.6f\n", _offsetAccelZ);

    cout << "Accel calibration complete" << endl;

    return _offsetAccelX, _offsetAccelY, _offsetAccelZ;
}

/* 3つのデータを格納しているポインタから加速度センサデータを取得する */
void MPU9250::getAccel(float* ax, float* ay, float* az)
{
    int16_t accel[3];

    getAccelCounts(&accel[0], &accel[1], &accel[2]);

    *ax = ((((float) accel[0]) * _accelScale) + _offsetAccelX) * G; // 型変換と値の縮尺
    *ay = ((((float) accel[1]) * _accelScale) + _offsetAccelY) * G;
    *az = ((((float) accel[2]) * _accelScale) + _offsetAccelZ) * G;
}


/* ジャイロセンサデータを取得して値を格納するポインタを取得し，データをカウントとして返す */
void MPU9250::getGyroCounts(int16_t* gx, int16_t* gy, int16_t* gz)
{
    char buff[6];
    int16_t gxx, gyy, gzz;

    readRegisters(GYRO_OUT, sizeof(buff), buff); // MPU9250からデータを取得する

    *gx = -(u2s((((int16_t)buff[0]) << 8) | buff[1])); // 16ビット値に変換
    *gy = -(u2s((((int16_t)buff[2]) << 8) | buff[3]));
    *gz = -(u2s((((int16_t)buff[4]) << 8) | buff[5]));
}

float MPU9250::calibGyro()
{
    cout << "Gyro calibration start" << endl;
    float sum[3] = {0, 0, 0};
    int16_t gyro[3];

    for (int i = 0; i < 1000; i++)
    {
        getGyroCounts(&gyro[0], &gyro[1], &gyro[2]);
        sum[0] += ((float)gyro[0]) * _gyroScale;
        sum[1] += ((float)gyro[1]) * _gyroScale;
        sum[2] += ((float)gyro[2]) * _gyroScale;
    }

    // 平均値をオフセットにする
    _offsetGyroX = -1.0 * sum[0] / 1000;
    _offsetGyroY = -1.0 * sum[1] / 1000;
    _offsetGyroZ = -1.0 * sum[2] / 1000;

    printf("offsetGyroX = %6.6f\t", _offsetGyroX);
    printf("offsetGyroY = %6.6f\t", _offsetGyroY);
    printf("offsetGyroZ = %6.6f\n", _offsetGyroZ);

    cout << "Gyro calibration complete" << endl;

    return _offsetGyroX, _offsetGyroY, _offsetGyroZ;
}

/* 3つのデータを格納しているポインタからジャイロセンサデータを取得する */
void MPU9250::getGyro(float* gx, float* gy, float* gz)
{
    int16_t gyro[3];
    float gxx = 0, gyy = 0, gzz = 0;
    float pregx = 0, pregy = 0, pregz = 0;
    float dt = 0.02;
    float rad = 3.1415926535 / 180;

    getGyroCounts(&gyro[0], &gyro[1], &gyro[2]);

    gxx = (((float) gyro[0]) * _gyroScale) + _offsetGyroX; // 型変換と値の縮尺
    gyy = (((float) gyro[1]) * _gyroScale) + _offsetGyroY;
    gzz = (((float) gyro[2]) * _gyroScale) + _offsetGyroZ;

    *gx += (((pregx + gxx) * dt) / 2) * rad;
    *gy += (((pregy + gyy) * dt) / 2) * rad;
    *gz += (((pregz + gzz) * dt) / 2) * rad;

    pregx = gxx;
    pregy = gyy;
    pregz = gzz;
}

// 変換行列
/* 加速度センサの軸とジャイロセンサの軸を変換して地磁気センサの軸に一致させる */
const int16_t MPU9250::tX[3] = {0, 1, 0};
const int16_t MPU9250::tY[3] = {1, 0, 0};
const int16_t MPU9250::tZ[3] = {0, 0, -1};


/* 地磁気センサデータを取得して値を格納するポインタを取得し，データをカウントとして返す */
void MPU9250::getMagCounts(int16_t* hx, int16_t* hy, int16_t* hz)
{
    char buff[7];
    int16_t hxx, hyy, hzz;

    //地磁気センサデータを外部センサバッファから読み出す
    readAK8953Registers(AK8963_ST1, 1, buff);
    if((buff[0] & 0x02) == 0x02)
    {
        //データオーバーランがあるので再度センシング
        i2c_read_byte_data(pi, _AK8963_handle, 0x09);
    }

    //ST1レジスタを確認してデータ読み出しが可能か確認する
    readAK8953Registers(AK8963_ST1, 1, buff);
    while((buff[0] & 0x01) != 0x01)
    {
        time_sleep(0.01);
        readAK8953Registers(AK8963_ST1, 1, buff);
    }

    readAK8953Registers(AK8963_HXL, sizeof(buff), buff);
    if( buff[6] == 0x10 )
    {
        hxx = u2s((((int16_t)buff[1]) << 8) | buff[0]); // 16ビット値に変換
        hyy = u2s((((int16_t)buff[3]) << 8) | buff[2]);
        hzz = u2s((((int16_t)buff[5]) << 8) | buff[4]);

        *hx = -(tX[0]*hxx + tX[1]*hyy + tX[2]*hzz); // 軸の変換
        *hy = -(tY[0]*hxx + tY[1]*hyy + tY[2]*hzz);
        *hz = tZ[0]*hxx + tZ[1]*hyy + tZ[2]*hzz;
    }
    else
    {
        *hx = 0;
        *hy = 0;
        *hz = 0;
    }
}


/* 3つのデータを格納しているポインタから地磁気センサデータを取得する */
void MPU9250::getMag(float* hx, float* hy, float* hz)
{
    int16_t mag[3];

    getMagCounts(&mag[0], &mag[1], &mag[2]);

    *hx = ((float) mag[0]) * _magScaleX; // 型変換と値の縮尺
    *hy = ((float) mag[1]) * _magScaleY;
    *hz = ((float) mag[2]) * _magScaleZ;
}


/* 温度センサデータを取得して値を格納するポインタを取得し，データをカウントとして返す */
void MPU9250::getTempCounts(int16_t* t)
{
    char buff[2];

    readRegisters(TEMP_OUT, sizeof(buff), buff); // MPU9250からデータを取得する

    *t = u2s((((int16_t)buff[0]) << 8) | buff[1]); // 16ビット値に変換
}


/* データを格納しているポインタから温度センサデータを取得する */
void MPU9250::getTemp(float *t)
{
    int16_t tempCount;

    getTempCounts(&tempCount);

    *t = (( ((float) tempCount) - _tempOffset )/_tempScale) + _tempOffset;
}

/* 2進数の補数を求める(unsingedデータをsignedデータに変換) */
int MPU9250::u2s(unsigned unsigneddata)
{
    if(unsigneddata & (0x01 << 15))
    {
        unsigneddata = -1 * ((unsigneddata ^ 0xffff) + 1);
    }
    return unsigneddata;
}


/* レジスタアドレスとデータを指定してMPU8259のレジスタに1バイトを書き込む */
bool MPU9250::writeRegister(uint8_t subAddress, uint8_t data)
{
    char buff[1];

    i2c_write_byte_data(pi, _i2c_handle, subAddress, data);

    time_sleep(0.01); // MPU9250への書き込み速度を遅くする必要がある

        /* レジスタを読み戻す */
        readRegisters(subAddress, sizeof(buff), buff);

    /* 書き込んだレジスタに対して読み戻したレジスタが同じかチェックする */
    return buff[0] == data;
}


/* 開始レジスタアドレス，バイト数，およびデタを格納するポインタを指定してMPU9250からレジスタの値を読み出す */
void MPU9250::readRegisters(uint8_t subAddress, uint8_t count, char* buf)
{
    i2c_read_i2c_block_data(pi, _i2c_handle, subAddress, buf, count);
}


/* レジスタアドレスとデータを指定してAK8963のレジスタに1バイトを書き込む */
bool MPU9250::writeAK8963Register(uint8_t subAddress, uint8_t data)
{
    i2c_write_byte_data(pi, _AK8963_handle, subAddress, data);

    time_sleep(0.01); // MPU9250への書き込み速度を遅くする必要がある
}


/* AK8963からレジスタを読み込む */
void MPU9250::readAK8953Registers(uint8_t subAddress, uint8_t count, char* buf)
{
    i2c_read_i2c_block_data(pi, _AK8963_handle, subAddress, buf, count);
}


/* 0x71になると予想されるMPU250 WHO_AM_Iレジスタ値を取得する */
uint8_t MPU9250::whoAmI()
{
    char buff[1];

    // WHO AM Iレジスタを読む
    readRegisters(WHO_AM_I, sizeof(buff), buff);

    // レジスタ値を返す
    return buff[0];
}


/*0x48になると予想されるAK8963 WHO_AM_Iレジスタ値を取得する */
uint8_t MPU9250::whoAmIAK8963()
{
    char buff[1];

    // WHO AM Iレジスタを読む
    readAK8953Registers(AK8963_WHO_AM_I, sizeof(buff), buff);

    // レジスタ値を返す
    return buff[0];
}
