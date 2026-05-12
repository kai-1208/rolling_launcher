#include "mbed.h"
#include "FP.hpp"
#include "pid.hpp"
#include <atomic>

CAN can1(PA_11, PA_12, (int)1e6);
DigitalIn bt1(BUTTON1);

FP fp(35, can1);

int32_t curr_enc[4] = {0};
bool is_first_read[4] = {true, true, true, true};
int32_t prev_enc[4] = {0};

// 1回転あたりのカウント数に合わせて調整してください。
constexpr float COUNTS_PER_REV = 4096.0f;
constexpr float SAMPLE_PERIOD_SEC = 0.02f;

// std::atomic<int16_t> pwm[4] = {0};

// 最新の計測rpm
float latest_rpm[4] = {0};
// 目標rpm
float target_rpm[4] = {0};

constexpr float KP = 0.7f;
constexpr float KI = 0.0f;
constexpr float KD = 0.0f;

float integral_err[4] = {0};
float prev_err[4] = {0};

// pwmの上限（調整要）
constexpr int32_t PWM_MAX = 16000;
constexpr int32_t PWM_MIN = -16000;

static int32_t clamp_pwm(int32_t v){
    if (v > PWM_MAX) return PWM_MAX;
    if (v < PWM_MIN) return PWM_MIN;
    return v;
}

PID pid0(KP, KI, KD, PID::Mode::VELOCITY);
PID pid1(KP, KI, KD, PID::Mode::VELOCITY);

int32_t pwm_manual[4] = {0};

// true: pid(rpm)制御を使う, false: pwm制御
bool pid_enabled = false;

void button_read() {
    while (true) {
        if (bt1 == 0) {
            if (pid_enabled) {
                target_rpm[0] = 10.0f;
                target_rpm[1] = 0.0f;
            } else {
                pwm_manual[0] = 10000;
                pwm_manual[1] = 6000;
            }
        } else {
            if (pid_enabled) {
                target_rpm[0] = 0.0f;
                target_rpm[1] = 0.0f;
            } else {
                pwm_manual[0] = 0;
                pwm_manual[1] = 0;
            }
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
                    prev_enc[motor_idx] = fp.receive[motor_idx].enc;
                    is_first_read[motor_idx] = false; 
                }
            }
            
            // FPライブラリからエンコーダ値を取得
            int32_t enc0 = fp.receive[0].enc;
            int32_t enc1 = fp.receive[1].enc;

            // 前回値との差分からrpmを計算
            float rpm0 = (enc0 - prev_enc[0]) * 60.0f / (COUNTS_PER_REV * SAMPLE_PERIOD_SEC);
            float rpm1 = (enc1 - prev_enc[1]) * 60.0f / (COUNTS_PER_REV * SAMPLE_PERIOD_SEC);
            prev_enc[0] = enc0;
            prev_enc[1] = enc1;
            latest_rpm[0] = rpm0;
            latest_rpm[1] = rpm1;
            
            // それぞれの初期値を引く
            printf("Encoder0: %ld, RPM0: %.1f, Encoder1: %ld, RPM1: %.1f\n",
                   enc0 - curr_enc[0], rpm0,
                   enc1 - curr_enc[1], rpm1);
        } else {
            // CANデータの読み取りがないときの処理が必要であればここに書く
        }
        ThisThread::sleep_for(20ms);
    }
}

// rpm制御pid
void rpm_control() {
    // PID の基本設定
    pid0.set_dt(SAMPLE_PERIOD_SEC);
    pid1.set_dt(SAMPLE_PERIOD_SEC);
    pid0.set_output_limits((float)PWM_MIN, (float)PWM_MAX);
    pid1.set_output_limits((float)PWM_MIN, (float)PWM_MAX);
    pid0.set_mode(PID::Mode::VELOCITY);
    pid1.set_mode(PID::Mode::VELOCITY);

    while (true) {
        if (pid_enabled) {
            pid0.set_goal(target_rpm[0]);
            float out0 = pid0.do_pid(latest_rpm[0]);
            int32_t pwm0 = clamp_pwm((int32_t)out0);
            fp.pwm[0] = (int16_t)pwm0;

            pid1.set_goal(target_rpm[1]);
            float out1 = pid1.do_pid(latest_rpm[1]);
            int32_t pwm1 = clamp_pwm((int32_t)out1);
            fp.pwm[1] = (int16_t)pwm1;
        } else {
            fp.pwm[0] = (int16_t)clamp_pwm(pwm_manual[0]);
            fp.pwm[1] = (int16_t)clamp_pwm(pwm_manual[1]);
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
    Thread rpm_control_thread;
    rpm_control_thread.start(rpm_control);
    while (true) {
        // ThisThread::sleep_for(1s);
    }
}