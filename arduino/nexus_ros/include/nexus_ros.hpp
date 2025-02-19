#ifndef NEXUS_ROS_H
#define NEXUS_ROS_H

#define _NAMIKI_MOTOR	 //for Namiki 22CL-103501PG80:1

#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <MotorWheel.h>
#include <Omni4WD.h>

irqISR(irq1,isr1);
MotorWheel wheel1(3,2,4,5,&irq1);

irqISR(irq2,isr2);
MotorWheel wheel2(11,12,14,15,&irq2);

irqISR(irq3,isr3);
MotorWheel wheel3(9,8,16,17,&irq3);

irqISR(irq4,isr4);
MotorWheel wheel4(10,7,18,19,&irq4);


/**
 * @brief NexusRos クラス (方法C)
 * 
 * - 4輪オムニロボットに対して (vx, vy, w) 速度指令を与える
 * - 各ホイールの実速度からオドメトリ(x, y, theta)を推定する
 * - Omni4WD ライブラリの「CarStat」や「setCarXXX()」系は使わず、
 *   直接 wheelXxxSetSpeedMMPS() を呼んで制御する
 */
class NexusRos
{
public:
    /**
     * @brief コンストラクタ
     * 
     * @param omni  Omni4WD のポインタ（外部で生成済みのものを渡す）
     * @param wheelRadius  ロボット中心からホイール中心までの距離 R [mm]
     */
    NexusRos()
    {
    }
    float val = 0.0f;

    /**
     * @brief デストラクタ
     */
    ~NexusRos() {}

    /**
     * @brief 追加の初期化関数
     * 
     * - Omni4WD のPIDを有効化
     * - モータの配置を固定(MOTORS_FB など)
     * 
     * @param kc   PIDのPゲイン
     * @param taui PIDのIゲイン
     * @param taud PIDのDゲイン
     * @param interval PIDRegulateの内部周期[ms]
     */
    void init(float kc = 0.31f, float taui = 0.01f, float taud = 0.0f, unsigned int interval = 10)
    {
        
        TCCR1B = TCCR1B & 0xf8 | 0x01; 
        TCCR2B = TCCR2B & 0xf8 | 0x01;
        omni_ = new Omni4WD(&wheel1, &wheel2, &wheel3, &wheel4);

        // Omni4WD 側のPID初期化
        omni_->PIDEnable(kc, taui, taud, interval);

        // "MOTORS_FB" に切り替え (前後が正方向という設定)
        omni_->switchMotorsReset();
        omni_->setCarStop();
    }
    bool ts;

    void update(){

        float val = random(-150.0f, 150.0f);
    
        omni_->wheelULSetSpeedMMPS(val);

        omni_->delayMS(50);
        // Serial.println(val);
    }

    /**
     * @brief (vx, vy, w) を指令 (mm/s, mm/s, rad/s)
     * 
     * 内部で逆運動学を用いて各ホイールの設定速度を計算し、
     * `wheelXxxSetSpeedMMPS(速度, DIR_...)` を直接呼び出す。
     */
    void setVelocity(const unsigned int& v_ul ,const unsigned int& v_ll ,const unsigned int& v_lr ,const  unsigned int& v_ur)
    {
        if(!omni_) return;
        // 実際に Omni4WD::wheelULSetSpeedMMPS(...) などを呼ぶ
        // 符号がプラスなら DIR_ADVANCE、マイナスなら DIR_BACKOFF
        float val = -1.0f;
        // omni_->setMotorAll(1, false);
        omni_->wheelULSetSpeedMMPS(val);
        // setWheelSpeedMMPS(v_ul, &Omni4WD::wheelULSetSpeedMMPS);
        // setWheelSpeedMMPS(v_ll, &Omni4WD::wheelLLSetSpeedMMPS);
        // setWheelSpeedMMPS(v_lr, &Omni4WD::wheelLRSetSpeedMMPS);
        // setWheelSpeedMMPS(v_ur, &Omni4WD::wheelURSetSpeedMMPS);
        // omni_->getCarSpeedMMPS();
    }

    /**
     * @brief オドメトリ(位置と姿勢)の更新
     * 
     * 一定周期で呼び出すことを想定。
     * - 各ホイールの現在速度 (mm/s) を取得
     * - (vx, vy, w) に合成
     * - x_, y_, theta_ を積分
     * 
     * @param currentMillis Arduino標準の millis() など
     */
    void updateOdometry()
    {
        if(!omni_) return;
        pulse_ul_ = omni_->wheelULGetCurPulse();
        pulse_ll_ = omni_->wheelLLGetCurPulse();
        pulse_lr_ = omni_->wheelLRGetCurPulse();
        pulse_ur_ = omni_->wheelURGetCurPulse();
        // Serial.print("p1: "); Serial.print(pulse_ul_);
        // Serial.print(", p2: "); Serial.print(pulse_ll_);
        // Serial.print(", p3: "); Serial.print(pulse_lr_);
        // Serial.print(", p4: "); Serial.println(pulse_ur_);
        sendPulseDataBinary(pulse_ul_, pulse_ll_, pulse_lr_, pulse_ur_, 0xA0);
    }

    void sendPulseDataBinary(long p1, long p2, long p3, long p4, byte data_type)
    {
        // データ部のサイズ (4つの long → 16 バイト)
        const uint16_t data_length = 16;
        // 総パケットサイズ: STX + データ種類 + データ長(2B) + データ部 + チェックサム + ETX = 22 バイト
        const uint16_t total_size = 1 + 1 + 2 + data_length + 1 + 1; 
        byte buffer[total_size];
        uint16_t offset = 0;
        byte checksum = 0;  // チェックサムは加算した下位 1 バイトとする

        // 1) STX (チェックサム計算対象外)
        buffer[offset++] = 0x02;

        // 2) データ種類 (1 バイト)
        buffer[offset] = data_type;
        checksum += data_type;  // ここで同時に加算
        offset++;

        // 3) データ長 (2 バイト, リトルエンディアン)
        buffer[offset] = (byte)(data_length & 0xFF);
        checksum += buffer[offset];
        offset++;
        buffer[offset] = (byte)((data_length >> 8) & 0xFF);
        checksum += buffer[offset];
        offset++;

        // 4) データ部: 4 つの long を個別にアンローリングして格納
        union {
            long val;
            byte b[4];
        } converter;

        // p1 の処理
        converter.val = p1;
        buffer[offset] = converter.b[0]; checksum += converter.b[0]; offset++;
        buffer[offset] = converter.b[1]; checksum += converter.b[1]; offset++;
        buffer[offset] = converter.b[2]; checksum += converter.b[2]; offset++;
        buffer[offset] = converter.b[3]; checksum += converter.b[3]; offset++;

        // p2 の処理
        converter.val = p2;
        buffer[offset] = converter.b[0]; checksum += converter.b[0]; offset++;
        buffer[offset] = converter.b[1]; checksum += converter.b[1]; offset++;
        buffer[offset] = converter.b[2]; checksum += converter.b[2]; offset++;
        buffer[offset] = converter.b[3]; checksum += converter.b[3]; offset++;

        // p3 の処理
        converter.val = p3;
        buffer[offset] = converter.b[0]; checksum += converter.b[0]; offset++;
        buffer[offset] = converter.b[1]; checksum += converter.b[1]; offset++;
        buffer[offset] = converter.b[2]; checksum += converter.b[2]; offset++;
        buffer[offset] = converter.b[3]; checksum += converter.b[3]; offset++;

        // p4 の処理
        converter.val = p4;
        buffer[offset] = converter.b[0]; checksum += converter.b[0]; offset++;
        buffer[offset] = converter.b[1]; checksum += converter.b[1]; offset++;
        buffer[offset] = converter.b[2]; checksum += converter.b[2]; offset++;
        buffer[offset] = converter.b[3]; checksum += converter.b[3]; offset++;

        // 5) チェックサム (上記で逐次計算済み)
        buffer[offset++] = checksum;

        // 6) ETX
        buffer[offset++] = 0x03;

        // 完成したパケットを一括送信
        // Serial.write(buffer, total_size);
    }

    /**
     * @brief 非常停止
     */
    void stop()
    {
        if(!omni_) return;
        omni_->setCarStop();  // ライブラリ標準の全輪ストップ関数
    }

    // --- オドメトリ取得 ---

private:
    /**
     * @brief ホイール別スピード設定を行う下位関数
     * 
     * 速度の符号を見て DIR_ADVANCE / DIR_BACKOFF を切り替え、
     * 絶対値を Omni4WD::wheelXxxSetSpeedMMPS(...) に渡す。
     */
    void setWheelSpeedMMPS(unsigned int speedValue,
                           unsigned int (Omni4WD::*wheelFunc)(unsigned int))
    {
        if(!omni_) return;
        float val = 10.0f;
        Serial.print("Speed : ");
        Serial.println(val);
        // 実際に呼び出し (例: (omni_->*wheelFunc)(spd, dir); )
        (omni_->*wheelFunc)(val);
    }

private:
    Omni4WD* omni_;      ///< Omni4WD インスタンス（外部管理）
    long pulse_ul_, pulse_ll_, pulse_ur_, pulse_lr_;

    unsigned long lastUpdateMillis_; ///< オドメトリ更新用に記録する時刻
};

#endif // NEXUS_ROS_H
