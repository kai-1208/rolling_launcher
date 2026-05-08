#include "mbed.h"
#include "FP.hpp"
#include <atomic>

BufferedSerial pc(USBTX, USBRX, 115200);
CAN can1(PA_11, PA_12, (int)1e6);
DigitalIn bt1(BUTTON1);

FP fp(35, can1);

int32_t curr_enc[4] = {0};
bool is_first_read[4] = {true, true, true, true};

// std::atomic<int16_t> pwm[4] = {0};

void button_read() {
    while (true) {
        if (bt1 == 0) {
            fp.pwm[0] = 7000;
            fp.pwm[1] = 0;
            printf("Button pressed\n"); // デバッグ用
        } else {
            fp.pwm[0] = 0;
            fp.pwm[1] = 0;
            // fp.pwm[2] = 0;
        }
        ThisThread::sleep_for(10ms);
    }
}

void can_send() {
    while (true) {
        fp.send();
        ThisThread::sleep_for(10ms);
    }
}

void can_receive() {
    while (true) {
        CANMessage enc_msg;
        if (can1.read(enc_msg)) {
            // 受信データをFPライブラリに反映
            fp.read(enc_msg);
            
            // 受信したメッセージがどのモーターか判定 (0〜3)
            // FPの読み取り規則に基づき、メッセージIDから判断します
            int motor_idx = enc_msg.id - fp.send_id - 1;
            
            // 該当するモーターが初回受信だった場合のみ初期値をセットする
            if (motor_idx >= 0 && motor_idx < 4) {
                if (is_first_read[motor_idx]) {
                    curr_enc[motor_idx] = fp.receive[motor_idx].enc;
                    is_first_read[motor_idx] = false; 
                }
            }
            
            // FPライブラリからエンコーダ値を取得
            int32_t enc0 = fp.receive[0].enc;
            int32_t enc1 = fp.receive[1].enc;
            
            // それぞれの初期値を引いて相対値(0から始まる値)にして表示
            printf("Encoder0: %ld, Encoder1: %ld\n", enc0 - curr_enc[0], enc1 - curr_enc[1]);
        } else {
            // CANデータの読み取りがないときの処理が必要であればここに書く
        }
        ThisThread::sleep_for(20ms);
    }
}

int main() {
    bt1.mode(PullUp);
    Thread button_thread;
    button_thread.start(button_read);
    Thread can_thread;
    can_thread.start(can_send);
    // Thread can_receive_thread;
    // can_receive_thread.start(can_receive);
    while (true) {
        // ThisThread::sleep_for(1s);
    }
}