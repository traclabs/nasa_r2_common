#include "nasa_r2_common_msgs/RosMsgPrinter.h"

std::string RosMsgPrinter::powerStateToString(const nasa_r2_common_msgs::PowerState& powerState)
{
    switch (powerState.data)
    {
        case nasa_r2_common_msgs::PowerState::LOGIC_POWER:
            return "LOGIC_POWER";

        case nasa_r2_common_msgs::PowerState::MOTOR48_POWER:
            return "MOTOR48_POWER";

        case nasa_r2_common_msgs::PowerState::MOTOR96_POWER:
            return "MOTOR96_POWER";

        case nasa_r2_common_msgs::PowerState::MOTOR_POWER:
            return "MOTOR_POWER";

        case nasa_r2_common_msgs::PowerState::POWER_OFF:
            return "POWER_OFF";

        case nasa_r2_common_msgs::PowerState::TRANSITIONING:
            return "TRANSITIONING";

        default:
            return "UNKNOWN";
    }
}

std::string RosMsgPrinter::jointControlModeToString(const nasa_r2_common_msgs::JointControlMode& controlMode)
{
    switch (controlMode.state)
    {
        case nasa_r2_common_msgs::JointControlMode::IGNORE:
            return "IGNORE";

        case nasa_r2_common_msgs::JointControlMode::INVALID:
            return "INVALID";

        case nasa_r2_common_msgs::JointControlMode::BOOTLOADER:
            return "BOOTLOADER";

        case nasa_r2_common_msgs::JointControlMode::FAULTED:
            return "FAULTED";

        case nasa_r2_common_msgs::JointControlMode::OFF:
            return "OFF";

        case nasa_r2_common_msgs::JointControlMode::PARK:
            return "PARK";

        case nasa_r2_common_msgs::JointControlMode::NEUTRAL:
            return "NEUTRAL";

        case nasa_r2_common_msgs::JointControlMode::DRIVE:
            return "DRIVE";

        default:
            return "UNDEFINED";
    }
}

std::string RosMsgPrinter::jointControlCommandModeToString(const nasa_r2_common_msgs::JointControlCommandMode& commandMode)
{
    switch (commandMode.state)
    {
        case nasa_r2_common_msgs::JointControlCommandMode::IGNORE:
            return "IGNORE";

        case nasa_r2_common_msgs::JointControlCommandMode::INVALID:
            return "INVALID";

        case nasa_r2_common_msgs::JointControlCommandMode::MOTCOM:
            return "MOTCOM";

        case nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSTEP:
            return "MULTILOOPSTEP";

        case nasa_r2_common_msgs::JointControlCommandMode::MULTILOOPSMOOTH:
            return "MULTILOOPSMOOTH";

        case nasa_r2_common_msgs::JointControlCommandMode::ACTUATOR:
            return "ACTUATOR";

        default:
            return "UNDEFINED";
    }
}

std::string RosMsgPrinter::jointControlCalibrationModeToString(const nasa_r2_common_msgs::JointControlCalibrationMode& calibrationMode)
{
    switch (calibrationMode.state)
    {
        case nasa_r2_common_msgs::JointControlCalibrationMode::IGNORE:
            return "IGNORE";

        case nasa_r2_common_msgs::JointControlCalibrationMode::DISABLE:
            return "DISABLE";

        case nasa_r2_common_msgs::JointControlCalibrationMode::ENABLE:
            return "ENABLE";

        case nasa_r2_common_msgs::JointControlCalibrationMode::UNCALIBRATED:
            return "UNCALIBRATED";

        default:
            return "UNDEFINED";
    }
}

std::string RosMsgPrinter::jointControlClearFaultModeToString(const nasa_r2_common_msgs::JointControlClearFaultMode& clearFaultMode)
{
    switch (clearFaultMode.state)
    {
        case nasa_r2_common_msgs::JointControlClearFaultMode::IGNORE:
            return "IGNORE";

        case nasa_r2_common_msgs::JointControlClearFaultMode::DISABLE:
            return "DISABLE";

        case nasa_r2_common_msgs::JointControlClearFaultMode::ENABLE:
            return "ENABLE";

        default:
            return "UNDEFINED";
    }
}

std::string RosMsgPrinter::jointControlCoeffsLoadedStateToString(const nasa_r2_common_msgs::JointControlCoeffState& coeffState)
{
    switch (coeffState.state)
    {
        case nasa_r2_common_msgs::JointControlCoeffState::LOADED:
            return "LOADED";

        case nasa_r2_common_msgs::JointControlCoeffState::NOTLOADED:
            return "NOTLOADED";

        default:
            return "UNDEFINED";
    }
}
