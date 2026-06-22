// Copyright 2025 Enactic, Inc.
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

#include <damiao_motor/dm_motor.hpp>
#include <damiao_motor/dm_motor_constants.hpp>

namespace damiao_motor {

// Constructor
Motor::Motor(const LimitParam& limits, uint32_t send_can_id, uint32_t recv_can_id, bool use_custom_firmware)
    : send_can_id_(send_can_id),
      recv_can_id_(recv_can_id),
    limits_(limits),
      custom_firmware_(use_custom_firmware),
      enabled_(false),
      state_q_(0.0),
      state_dq_(0.0),
      state_tau_(0.0),
      state_tmos_(0),
      state_trotor_(0) {}

// Enable methods
void Motor::set_enabled(bool enable) { this->enabled_ = enable; }

// Parameter methods
// TODO: storing temp params in motor object might not be a good idea
// also -1 is not a good default value, consider using a different value
double Motor::get_param(int RID) const {
    auto it = temp_param_dict_.find(RID);
    return (it != temp_param_dict_.end()) ? it->second : -1;
}

void Motor::set_temp_param(int RID, double val) { temp_param_dict_[RID] = val; }

// State update methods
void Motor::update_state(double q, double dq, double tau, int tmos, int trotor) {
    state_q_ = q;
    state_dq_ = dq;
    state_tau_ = tau;
    state_tmos_ = tmos;
    state_trotor_ = trotor;
}

void Motor::set_state_tmos(int tmos) { state_tmos_ = tmos; }

void Motor::set_state_trotor(int trotor) { state_trotor_ = trotor; }

}  // namespace damiao_motor
