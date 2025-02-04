#include <Arduino.h>

// ライブラリヘッダ
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#include <MotorWheel.h>
#include <Omni4WD.h>

// 先ほどのクラス
#include "include/nexus_ros.hpp"

//----------- あなたの配線に合わせて IRQピンや PWMピンなどを設定 -------------//
irqISR(irq1,isr1);
MotorWheel wheel1(3,  2,  4,  5,  &irq1);
irqISR(irq2,isr2);
MotorWheel wheel2(11, 12, 14, 15, &irq2);
irqISR(irq3,isr3);
MotorWheel wheel3(9,  8,  16, 17, &irq3);
irqISR(irq4,isr4);
MotorWheel wheel4(10, 7,  18, 19, &irq4);

// Omni4WDインスタンス
Omni4WD omni(&wheel1, &wheel2, &wheel3, &wheel4);

// ロボット中心～ホイール中心までの距離[mm] (実機の寸法に合わせて)
static const float ROBOT_RADIUS = 100.0f;

// NexusRos (方法C)
NexusRos nexus(&omni, ROBOT_RADIUS);

void setup()
{
  Serial.begin(115200);

  // タイマ設定 (PWM周波数を31.25kHzにする例)
  // Timer0は触らない(→millis()維持)、Timer1, Timer2だけ設定
  // ここはサンプル通り必要に応じて変更
  TCCR1B = TCCR1B & 0xf8 | 0x01; 
  TCCR2B = TCCR2B & 0xf8 | 0x01;

  // NexusRos初期化 (内部で omni_->PIDEnable(...) も呼ぶ)
  nexus.init(0.31f, 0.01f, 0.0f, 10);
}

void loop()
{
  /***********************************************************
   * 例: (vx, vy, w) = (100, 0, 0.5) としてロボットを回転させながら前進
   *  単位: vx,vy = [mm/s], w = [rad/s]
   ***********************************************************/
  float vx = 100.0f;
  float vy =   0.0f;
  float w  =   0.5f;

  // 1) 目標速度をセット (→ wheelXxxSetSpeedMMPSを呼ぶ)
  nexus.setVelocity(vx, vy, w);

  // 2) ライブラリの delayMS(...) を呼び、内部で PIDRegulate() を動かす
  //    ここでは 10ms 毎に呼ぶ
  omni.delayMS(10);

  // 3) オドメトリ更新
  nexus.updateOdometry(millis());

  // 4) デバッグ表示
  float x = nexus.getX();     
  float y = nexus.getY();     
  float th= nexus.getTheta(); 
  Serial.print("x=");  Serial.print(x);
  Serial.print(" y="); Serial.print(y);
  Serial.print(" th=");Serial.println(th);

  // 多少待ってループ
  // (実質 omni.delayMS(10) の間に10msかかるので、ここでは少ない待ち)
  delay(5);
}
