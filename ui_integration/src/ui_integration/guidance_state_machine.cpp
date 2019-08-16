/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "ui_integration/guidance_state_machine.hpp"

namespace ui_integration
{
    void GuidanceStateMachine::onGuidanceSignal(Signal signal)
    {
        switch(currentGuidanceState)
        {
            case State::STARTUP:
                if(signal == Signal::INITIALIZED)
                {
                    currentGuidanceState = State::DRIVERS_READY;
                } else if(signal == Signal::SHUTDOWN)
                {
                    currentGuidanceState = State::SHUTDOWN;
                }
                break;
            case State::DRIVERS_READY:
                if(signal == Signal::ACTIVATED)
                {
                    currentGuidanceState = State::ACTIVE;
                } else if(signal == Signal::SHUTDOWN)
                {
                    currentGuidanceState = State::SHUTDOWN;
                }
                break;
            case State::ACTIVE:
                if(signal == Signal::ENGAGED)
                {
                    currentGuidanceState = State::ENGAGED;
                } else if(signal == Signal::DISENGAGED)
                {
                    currentGuidanceState = State::DRIVERS_READY;
                } else if(signal == Signal::SHUTDOWN)
                {
                    currentGuidanceState = State::SHUTDOWN;
                }
                break;
            case State::ENGAGED:
                if(signal == Signal::DISENGAGED)
                {
                    currentGuidanceState = State::DRIVERS_READY;
                } else if(signal == Signal::SHUTDOWN)
                {
                    currentGuidanceState = State::SHUTDOWN;
                }
                break;
            case State::SHUTDOWN:
                break;
        }
    }

    void GuidanceStateMachine::onSystemAlert(const cav_msgs::SystemAlertConstPtr& msg)
    {
        if(msg->type == msg->DRIVERS_READY)
        {
            onGuidanceSignal(Signal::INITIALIZED);
        } else if(msg->type == msg->SHUTDOWN || msg->type == msg->FATAL)
        {
            onGuidanceSignal(Signal::SHUTDOWN);
        }
    }

    void GuidanceStateMachine::onSetGuidanceActive(bool msg)
    {
        if(msg)
        {
            onGuidanceSignal(Signal::ACTIVATED);
        }
    }

    void GuidanceStateMachine::onRoboticStatus(const cav_msgs::RobotEnabledConstPtr& msg)
    {
        if(msg->robot_enabled && msg->robot_active)
        {
            onGuidanceSignal(Signal::ENGAGED);
        } else if(msg->robot_enabled && !msg->robot_active)
        {
            onGuidanceSignal(Signal::DISENGAGED);
        }
    }

    uint8_t GuidanceStateMachine::getCurrentState()
    {
        return static_cast<uint8_t>(currentGuidanceState);
    }

    GuidanceStateMachine::GuidanceStateMachine() : currentGuidanceState(State::STARTUP) {}

}