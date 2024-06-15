#include "mbed.h"
#include "PID.hpp"

uint8_t DATA[8] = {};
InterruptIn button(PC_8);       // リミットスイッチ
InterruptIn button_reset(PC_9); // 起動スイッチ
int DJI_ID = 0x200;

int16_t speed = 0; // 16ビットの最大値
int16_t sokudo = 0;
int16_t mokuhyou = 0;
const float kp = 0.1;
const float ki = 0.035;
const float kd = 0.0;
const float sample_time = 0.02; // 20ms sample time
PID pid_controller(kp, ki, kd, sample_time);

bool flag = false;

int main()
{
    speed = 0; // 速度0
    for (int i = 0; i < 8; i++)
    {
        DATA[i] = 0;
    }
    button_reset.mode(PullUp);
    BufferedSerial pc(USBTX, USBRX, 115200);
    CAN can(PA_11, PA_12, (int)1e6);
    CANMessage msg;

    while (1)
    {
        auto now = HighResClock::now(); // タイマー設定
        static auto pre = now;
        bool sw = button_reset.read(); // ボタン読み取り

        if (now - pre > 1000ms && sw == 0)
        {
            printf("Restart\n");
            speed = 16000; // 速度MAX
            mokuhyou = 12543;
            float output = pid_controller.calculate(mokuhyou, sokudo);
            printf("speed=%f\n", output);
            int16_t mokuhyou_int16 = static_cast<int16_t>(output);
            for (int i = 0; i < 8; i += 2)
            {
                DATA[i] = (mokuhyou_int16 >> 8) & 0xFF; // 上位バイト
                DATA[i + 1] = mokuhyou_int16 & 0xFF;    // 下位バイト
            }
            mokuhyou_int16 = 0;
            sokudo = 0;
            output = 0;
            pre = now; // タイマーリセット
            flag = true;
        }

        if (flag == true && now - pre > 1000ms)
        {
            speed = 0; // 速度0
            for (int i = 0; i < 8; i += 2)
            {
                DATA[i] = (speed >> 8) & 0xFF; // 上位バイト
                DATA[i + 1] = speed & 0xFF;    // 下位バイト
            }
            flag = false;
        }
        CANMessage msg(DJI_ID, DATA, 8);
        can.write(msg);
    }
}
