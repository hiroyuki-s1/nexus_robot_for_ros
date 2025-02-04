#ifndef NEXUS_ROS_H
#define NEXUS_ROS_H

#include <MotorWheel.h>
#include <Omni4WD.h>
#include <math.h>
#include <stdint.h>

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
    NexusRos(Omni4WD* omni, float wheelRadius)
        : omni_(omni)
        , R_(wheelRadius)
        , x_(0.0f)
        , y_(0.0f)
        , theta_(0.0f)
        , lastUpdateMillis_(0)
    {
    }

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
        if(!omni_) return;

        // Omni4WD 側のPID初期化
        omni_->PIDEnable(kc, taui, taud, interval);

        // "MOTORS_FB" に切り替え (前後が正方向という設定)
        omni_->switchMotorsReset();
    }

    /**
     * @brief (vx, vy, w) を指令 (mm/s, mm/s, rad/s)
     * 
     * 内部で逆運動学を用いて各ホイールの設定速度を計算し、
     * `wheelXxxSetSpeedMMPS(速度, DIR_...)` を直接呼び出す。
     */
    void setVelocity(float vx, float vy, float w)
    {
        if(!omni_) return;

        // 45°オフセット配置を仮定(ダイヤ型)
        // alpha = 1/sqrt(2)
        static const float alpha = 1.0f / (float)M_SQRT2;

        // 各ホイール目標速度 [mm/s]
        // ここで sign(速度) によって DIR_ADVANCE / DIR_BACKOFF を振り分けるため、
        // setWheelSpeedMMPS() の中で fabs() + 方向判定をする
        float v1 = alpha * (vx + vy) + R_ * w;   // UpperLeft
        float v2 = alpha * (-vx + vy) + R_ * w;  // LowerLeft
        float v3 = alpha * (-vx - vy) + R_ * w;  // LowerRight
        float v4 = alpha * ( vx - vy) + R_ * w;  // UpperRight

        // 実際に Omni4WD::wheelULSetSpeedMMPS(...) などを呼ぶ
        // 符号がプラスなら DIR_ADVANCE、マイナスなら DIR_BACKOFF
        setWheelSpeedMMPS(v1, &Omni4WD::wheelULSetSpeedMMPS);
        setWheelSpeedMMPS(v2, &Omni4WD::wheelLLSetSpeedMMPS);
        setWheelSpeedMMPS(v3, &Omni4WD::wheelLRSetSpeedMMPS);
        setWheelSpeedMMPS(v4, &Omni4WD::wheelURSetSpeedMMPS);
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
    void updateOdometry(unsigned long currentMillis)
    {
        if(!omni_) return;

        unsigned long dtMillis = (lastUpdateMillis_ == 0) ? 0 : (currentMillis - lastUpdateMillis_);
        lastUpdateMillis_ = currentMillis;
        if(dtMillis == 0) {
            // 初回呼び出しまたは時間差が0の場合はスキップ
            return;
        }
        float dt = (float)dtMillis * 0.001f; // 秒

        // 各ホイール実速度 [mm/s] を取得
        // ※ライブラリが正負の速度を返すかどうか注意
        //   MotorWheel::getSpeedMMPS() は正のみの可能性あり
        float v1 = (float)omni_->wheelULGetSpeedMMPS(); // UpperLeft
        float v2 = (float)omni_->wheelLLGetSpeedMMPS(); // LowerLeft
        float v3 = (float)omni_->wheelLRGetSpeedMMPS(); // LowerRight
        float v4 = (float)omni_->wheelURGetSpeedMMPS(); // UpperRight

        // ここでは仮に全て正値として扱い、向きは無視していますが
        // 実際には DIR_BACKOFF のときは負速度にするなど工夫が必要。
        // 例: if (omni_->_wheelUL->getDir()==DIR_BACKOFF) v1 = -v1; など

        // 逆方向の合成行列で (vx, vy, w) を復元
        static const float alpha = 1.0f / (float)M_SQRT2;
        // 下記は一例。ロボット軸の取り方や回転方向により符号が変わるかもしれない
        // 参考式:
        //   vx = 0.5 * alpha * (v1 - v2 - v3 + v4)
        //   vy = 0.5 * alpha * (v1 + v2 - v3 - v4)
        //   w  = (v1 + v2 + v3 + v4) / (4*R_)
        // (本来はマトリックスで行う)
        float vx = 0.5f * alpha * (v1 - v2 - v3 + v4);
        float vy = 0.5f * alpha * (v1 + v2 - v3 - v4);
        float sum = (v1 + v2 + v3 + v4);
        float w  = sum / (4.0f * R_); // rad/s

        // ロボット座標→世界座標への変換 (theta_ を用いる)
        float cosT = cosf(theta_);
        float sinT = sinf(theta_);

        float dx  = vx * cosT - vy * sinT;
        float dy  = vx * sinT + vy * cosT;
        float dth = w;

        x_     += dx * dt;
        y_     += dy * dt;
        theta_ += dth * dt;

        // theta_ の正規化が必要ならここで実施 (例: -π < theta_ <= +π)
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
    float getX() const     { return x_; }
    float getY() const     { return y_; }
    float getTheta() const { return theta_; }

private:
    /**
     * @brief ホイール別スピード設定を行う下位関数
     * 
     * 速度の符号を見て DIR_ADVANCE / DIR_BACKOFF を切り替え、
     * 絶対値を Omni4WD::wheelXxxSetSpeedMMPS(...) に渡す。
     */
    void setWheelSpeedMMPS(float speedValue,
                           unsigned int (Omni4WD::*wheelFunc)(unsigned int,bool))
    {
        if(!omni_) return;

        bool dir = (speedValue >= 0.0f) ? DIR_ADVANCE : DIR_BACKOFF;
        unsigned int spd = (unsigned int)fabs(speedValue);

        // 実際に呼び出し (例: (omni_->*wheelFunc)(spd, dir); )
        (omni_->*wheelFunc)(spd, dir);
    }

private:
    Omni4WD* omni_;      ///< Omni4WD インスタンス（外部管理）
    float    R_;         ///< 中心→ホイール中心までの距離[mm]

    float x_;            ///< オドメトリ推定 x[mm]
    float y_;            ///< オドメトリ推定 y[mm]
    float theta_;        ///< オドメトリ推定 θ[rad]

    unsigned long lastUpdateMillis_; ///< オドメトリ更新用に記録する時刻
};

#endif // NEXUS_ROS_H
