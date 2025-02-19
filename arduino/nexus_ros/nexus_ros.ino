#include <Arduino.h>

// ライブラリヘッダ
// 先ほどのクラス
#include "include/nexus_ros.hpp"


// Omni4WDインスタンス
unsigned int v = 10;
// ロボット中心～ホイール中心までの距離[mm] (実機の寸法に合わせて)

// NexusRos (方法C)
NexusRos nexus;

void setup()
{
  Serial.begin(115200);

  // タイマ設定 (PWM周波数を31.25kHzにする例)
  // Timer0は触らない(→millis()維持)、Timer1, Timer2だけ設定
  // ここはサンプル通り必要に応じて変更

  // NexusRos初期化 (内部で omni_->PIDEnable(...) も呼ぶ)
  nexus.init(0.31f, 0.01f, 0.0f, 10);
  delay(5000);
}

void loop()
{
  nexus.update();
  // omni.delayMS(1000);
  // unsigned int vel = 0;
  // nexus.setVelocity(vel, vel, vel, vel);
  // omni.delayMS(3000);
  // vel = 100;
  // nexus.setVelocity(vel, vel, vel, vel);
  // Serial.println(vel);
  // nexus.updateOdometry();// v++;
}
