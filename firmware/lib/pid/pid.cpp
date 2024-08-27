// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Arduino.h"
#include "pid.h"

PID::PID(float min_val, float max_val, float kp, float ki, float kd):
    min_val_(min_val),
    max_val_(max_val),
    kp_(kp),
    ki_(ki),
    kd_(kd)
{
}

double PID::compute(float setpoint, float measured_value)
// {
//     double error;
//     double pid;

//     //setpoint is constrained between min and max to prevent pid from having too much error
//     error = setpoint - measured_value;
//     integral_ += error;
//     derivative_ = error - prev_error_;

//     if(setpoint == 0 && error == 0)
//     {
//         integral_ = 0;
//         derivative_ = 0;
//     }

//     pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);
//     prev_error_ = error;

//     return constrain(pid, min_val_, max_val_);
// }
{
    double error = setpoint - measured_value;

    // Tích phân giới hạn để tránh windup
    integral_ += error;
    if (integral_ > max_val_) {
        integral_ = max_val_;
    } else if (integral_ < min_val_) {
        integral_ = min_val_;
    }

    // Lọc vi phân để giảm nhiễu
    double derivative = error - prev_error_;
    derivative_ = 0.9 * derivative_ + 0.1 * derivative;

// nếu hệ thống phản ứng quá chậm, giảm hệ số 0.9 và tăng 0.1 có thể giúp cải thiện độ nhạy.


    // Tính toán giá trị PID

    double pid = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);

    // Lưu lại sai số trước đó
    prev_error_ = error;

    // Giới hạn đầu ra (nếu cần thiết)
    if (pid > max_val_) {
        return max_val_;
    } else if (pid < min_val_) {
        return min_val_;
    } else {
        return pid;
    }
}
void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
