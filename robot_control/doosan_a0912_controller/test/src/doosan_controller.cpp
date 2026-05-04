#include "doosan_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <iostream>

namespace doosan
{

DoosanController *DoosanController::active_instance_ = nullptr;

namespace
{
float commandTimeOrDefault(float requested, float fallback)
{
    return requested > 0.0f ? requested : fallback;
}
}  // namespace

DoosanController::DoosanController()
{
    active_instance_ = this;
}

DoosanController::~DoosanController()
{
    shutdown();
    if (active_instance_ == this)
    {
        active_instance_ = nullptr;
    }
}

bool DoosanController::initialize(
    const Config &config,
    bool enable_realtime_control,
    std::chrono::seconds timeout)
{
    if (!connect(config))
    {
        return false;
    }

    if (!servoOn(timeout))
    {
        shutdown();
        return false;
    }

    if (enable_realtime_control && config_.system == RobotSystem::Real)
    {
        if (!connectRealtimeControl())
        {
            shutdown();
            return false;
        }
    }

    return true;
}

bool DoosanController::positionControl(
    const JointArray &position,
    const JointArray &velocity,
    const JointArray &acceleration,
    float command_time_sec)
{
    return positionControl(
        position,
        velocity,
        acceleration,
        config_.position_move_time_sec,
        command_time_sec);
}

bool DoosanController::positionControl(
    const JointArray &position,
    const JointArray &velocity,
    const JointArray &acceleration,
    float move_time_sec,
    float command_time_sec)
{
    if (!servo_on_)
    {
        std::cerr << "[robot] positionControl requires initialize()/servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] positionControl uses servoj_rt and requires real robot mode\n";
        return false;
    }

    const bool started_rt = !realtime_control_running_;
    if (!connectRealtimeControl())
    {
        return false;
    }

    if (!setJointPositionTrajectory(position, velocity, acceleration, move_time_sec, command_time_sec))
    {
        if (started_rt)
        {
            stopRealtimeControl();
        }
        return false;
    }

    if (!startRealtimeLoop())
    {
        stopRealtimeControl();
        return false;
    }

    if (started_rt && !startRealtimeControl())
    {
        realtime_loop_running_ = false;
        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }
        stopRealtimeControl();
        return false;
    }

    return true;
}

bool DoosanController::velocityControl(
    const JointArray &velocity,
    const JointArray &acceleration,
    float command_time_sec)
{
    if (!servo_on_)
    {
        std::cerr << "[robot] velocityControl requires initialize()/servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] velocityControl uses speedj_rt and requires real robot mode\n";
        return false;
    }

    const bool started_rt = !realtime_control_running_;
    if (!connectRealtimeControl())
    {
        return false;
    }

    setJointVelocityTarget(velocity, acceleration, command_time_sec);

    if (!startRealtimeLoop())
    {
        stopRealtimeControl();
        return false;
    }

    if (started_rt && !startRealtimeControl())
    {
        realtime_loop_running_ = false;
        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }
        stopRealtimeControl();
        return false;
    }

    return true;
}

bool DoosanController::velocityControlToPosition(
    const JointArray &target_position,
    const JointArray &max_velocity,
    const JointArray &acceleration,
    float gain,
    float tolerance_deg,
    float timeout_sec,
    float command_time_sec)
{
    if (!servo_on_)
    {
        std::cerr << "[robot] velocityControlToPosition requires initialize()/servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] velocityControlToPosition uses speedj_rt and requires real robot mode\n";
        return false;
    }

    const bool started_rt = !realtime_control_running_;
    if (!connectRealtimeControl())
    {
        return false;
    }

    if (!setJointVelocityPositionTarget(
            target_position,
            max_velocity,
            acceleration,
            gain,
            tolerance_deg,
            timeout_sec,
            command_time_sec))
    {
        if (started_rt)
        {
            stopRealtimeControl();
        }
        return false;
    }

    if (!startRealtimeLoop())
    {
        stopRealtimeControl();
        return false;
    }

    if (started_rt && !startRealtimeControl())
    {
        realtime_loop_running_ = false;
        if (realtime_thread_.joinable())
        {
            realtime_thread_.join();
        }
        stopRealtimeControl();
        return false;
    }

    return true;
}

bool DoosanController::connect(const Config &config)
{
    if (connected_)
    {
        return true;
    }

    config_ = config;
    if (config_.rt_ip.empty())
    {
        config_.rt_ip = config_.ip;
    }

    has_control_access_ = false;
    is_standby_ = false;
    disconnected_ = false;
    active_instance_ = this;

    {
        std::lock_guard<std::mutex> lock(drfl_mutex_);
        drfl_.set_on_monitoring_access_control(&DoosanController::onAccessControl);
        drfl_.set_on_monitoring_state(&DoosanController::onRobotState);
        drfl_.set_on_disconnected(&DoosanController::onDisconnected);

        std::cout << "[robot] open_connection ip=" << config_.ip << " port=" << config_.port << '\n';
        if (!drfl_.open_connection(config_.ip, config_.port))
        {
            std::cerr << "[robot] failed to open DRFL connection\n";
            return false;
        }

        SYSTEM_VERSION system_version;
        std::memset(&system_version, 0, sizeof(system_version));
        if (drfl_.get_system_version(&system_version))
        {
            std::cout << "[robot] DRCF version: " << system_version._szController << '\n';
            std::cout << "[robot] DRFL version: " << drfl_.get_library_version() << '\n';
        }

        if (!drfl_.setup_monitoring_version(1))
        {
            std::cerr << "[robot] setup_monitoring_version(1) failed\n";
            drfl_.close_connection();
            return false;
        }
    }

    connected_ = true;
    return true;
}

bool DoosanController::servoOn(std::chrono::seconds timeout)
{
    if (!connected_)
    {
        std::cerr << "[robot] servoOn requested before connect\n";
        return false;
    }

    if (!waitForControlAccessAndStandby(timeout))
    {
        std::cerr << "[robot] control authority/STANDBY timeout\n";
        return false;
    }

    const auto robot_system =
        config_.system == RobotSystem::Real ? ROBOT_SYSTEM_REAL : ROBOT_SYSTEM_VIRTUAL;

    std::lock_guard<std::mutex> lock(drfl_mutex_);
    std::cout << "[robot] set_robot_mode AUTONOMOUS\n";
    if (!drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS))
    {
        std::cerr << "[robot] failed to set ROBOT_MODE_AUTONOMOUS\n";
        return false;
    }

    std::cout << "[robot] set_robot_system " << robotSystemToString(config_.system) << '\n';
    if (!drfl_.set_robot_system(robot_system))
    {
        std::cerr << "[robot] failed to set robot system\n";
        return false;
    }

    drfl_.set_auto_servo_off(false, 5.0f);
    drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);

    servo_on_ = true;
    return true;
}

bool DoosanController::servoOff()
{
    if (!connected_)
    {
        return true;
    }

    stopRealtimeControl();

    std::lock_guard<std::mutex> lock(drfl_mutex_);
    std::cout << "[robot] servo_off\n";
    drfl_.servo_off(STOP_TYPE_QUICK);
    servo_on_ = false;
    return true;
}

bool DoosanController::connectRealtimeControl()
{
    if (!connected_ || !servo_on_)
    {
        std::cerr << "[robot] connectRealtimeControl requires connect() and servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] realtime control is only enabled for real robot mode\n";
        return false;
    }

    if (realtime_control_connected_)
    {
        return true;
    }

    std::lock_guard<std::mutex> lock(drfl_mutex_);
    std::cout << "[robot] connect_rt_control ip=" << config_.rt_ip
              << " port=" << config_.rt_port << '\n';
    if (!drfl_.connect_rt_control(config_.rt_ip, config_.rt_port))
    {
        std::cerr << "[robot] failed to connect RT control\n";
        return false;
    }
    realtime_control_connected_ = true;

    if (!drfl_.set_rt_control_output("v1.0", config_.rt_period_sec, config_.rt_loss_count))
    {
        std::cerr << "[robot] failed to set RT control output\n";
        return false;
    }

    auto velocity_limit = config_.rt_velocity_limit;
    auto acceleration_limit = config_.rt_acceleration_limit;
    if (!drfl_.set_velj_rt(velocity_limit.data()) ||
        !drfl_.set_accj_rt(acceleration_limit.data()))
    {
        std::cerr << "[robot] failed to set RT joint velocity/acceleration limits\n";
        return false;
    }

    return true;
}

bool DoosanController::startRealtimeControl()
{
    if (!connected_ || !servo_on_)
    {
        std::cerr << "[robot] startRealtimeControl requires connect() and servoOn()\n";
        return false;
    }

    if (config_.system != RobotSystem::Real)
    {
        std::cerr << "[robot] realtime control is only enabled for real robot mode\n";
        return false;
    }

    if (realtime_control_running_)
    {
        return true;
    }

    if (!connectRealtimeControl())
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(drfl_mutex_);
    if (!drfl_.start_rt_control())
    {
        std::cerr << "[robot] failed to start RT control\n";
        return false;
    }

    {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        if (command_.trajectory_active)
        {
            command_.trajectory_start = Clock::now();
        }
        if (command_.mode == CommandMode::JointVelocityTarget)
        {
            command_.velocity_target_start = Clock::now();
            command_.velocity_target_last_update = command_.velocity_target_start;
        }
    }

    realtime_control_running_ = true;
    return true;
}

void DoosanController::stopRealtimeControl()
{
    realtime_loop_running_ = false;

    if (realtime_thread_.joinable() &&
        realtime_thread_.get_id() != std::this_thread::get_id())
    {
        realtime_thread_.join();
    }

    stopRealtimeSession();
}

bool DoosanController::startRealtimeLoop()
{
    if (!realtime_control_connected_)
    {
        std::cerr << "[robot] startRealtimeLoop requires connectRealtimeControl()\n";
        return false;
    }

    if (realtime_loop_running_)
    {
        return true;
    }

    if (realtime_thread_.joinable())
    {
        realtime_thread_.join();
    }

    realtime_loop_running_ = true;
    try
    {
        realtime_thread_ = std::thread(&DoosanController::realtimeLoop, this);
    }
    catch (const std::exception &e)
    {
        realtime_loop_running_ = false;
        std::cerr << "[robot] failed to create realtime thread: " << e.what() << '\n';
        stopRealtimeControl();
        return false;
    }
    catch (...)
    {
        realtime_loop_running_ = false;
        std::cerr << "[robot] failed to create realtime thread\n";
        stopRealtimeControl();
        return false;
    }
    return true;
}

void DoosanController::stopRealtimeSession()
{
    if (!realtime_control_running_.exchange(false))
    {
        return;
    }

    std::lock_guard<std::mutex> lock(drfl_mutex_);
    drfl_.stop_rt_control();
}

void DoosanController::disconnectRealtimeControl()
{
    stopRealtimeControl();

    if (!realtime_control_connected_.exchange(false))
    {
        return;
    }

    std::lock_guard<std::mutex> lock(drfl_mutex_);
    drfl_.disconnect_rt_control();
}

void DoosanController::setJointPositionTarget(
    const JointArray &position,
    const JointArray &velocity,
    const JointArray &acceleration,
    float command_time_sec)
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_.mode = CommandMode::JointPosition;
    command_.position = position;
    command_.velocity = velocity;
    command_.acceleration = acceleration;
    command_.command_time_sec = commandTimeOrDefault(command_time_sec, config_.command_time_sec);
    command_.trajectory_active = false;
    command_.stop_rt_when_done = false;
    command_.target_position = position;
}

void DoosanController::setJointVelocityTarget(
    const JointArray &velocity,
    const JointArray &acceleration,
    float command_time_sec)
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_.mode = CommandMode::JointVelocity;
    command_.velocity = velocity;
    command_.acceleration = acceleration;
    command_.command_time_sec = commandTimeOrDefault(command_time_sec, config_.command_time_sec);
    command_.trajectory_active = false;
    command_.stop_rt_when_done = false;
}

void DoosanController::holdCommand()
{
    if (servo_on_ && realtime_control_running_ && config_.system == RobotSystem::Real)
    {
        if (holdCurrentPosition())
        {
            return;
        }

        std::cerr << "[robot] failed to read current joint state for hold command\n";
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    command_.mode = CommandMode::Hold;
}

void DoosanController::shutdown()
{
    if (connected_ && servo_on_)
    {
        servoOff();
    }
    else
    {
        stopRealtimeControl();
    }

    disconnectRealtimeControl();

    if (connected_)
    {
        std::lock_guard<std::mutex> lock(drfl_mutex_);
        drfl_.close_connection();
        connected_ = false;
        servo_on_ = false;
    }
}

bool DoosanController::isConnected() const
{
    return connected_;
}

bool DoosanController::isServoOn() const
{
    return servo_on_;
}

bool DoosanController::isRealtimeControlRunning() const
{
    return realtime_control_running_;
}

DoosanController::RobotSystem DoosanController::robotSystemFromString(const std::string &value)
{
    if (value == "virtual")
    {
        return RobotSystem::Virtual;
    }
    return RobotSystem::Real;
}

bool DoosanController::waitForControlAccessAndStandby(std::chrono::seconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < deadline && !disconnected_)
    {
        if (!has_control_access_)
        {
            std::lock_guard<std::mutex> lock(drfl_mutex_);
            std::cout << "[robot] request control access\n";
            drfl_.manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        }
        else if (!is_standby_)
        {
            std::lock_guard<std::mutex> lock(drfl_mutex_);
            std::cout << "[robot] servo on\n";
            drfl_.set_robot_control(CONTROL_SERVO_ON);
        }
        else
        {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return false;
}

bool DoosanController::readCurrentJointState(JointArray *position, JointArray *velocity)
{
    std::lock_guard<std::mutex> lock(drfl_mutex_);
    if (realtime_control_running_)
    {
        auto *data = drfl_.read_data_rt();
        bool rt_data_valid = data && std::isfinite(data->time_stamp) && data->time_stamp > 0.0;
        if (rt_data_valid && position)
        {
            for (int i = 0; i < kNumJoints; ++i)
            {
                rt_data_valid = rt_data_valid && std::isfinite(data->actual_joint_position[i]);
            }
        }
        if (rt_data_valid && velocity)
        {
            for (int i = 0; i < kNumJoints; ++i)
            {
                rt_data_valid = rt_data_valid && std::isfinite(data->actual_joint_velocity[i]);
            }
        }

        if (rt_data_valid)
        {
            if (position)
            {
                for (int i = 0; i < kNumJoints; ++i)
                {
                    (*position)[i] = data->actual_joint_position[i];
                }
            }

            if (velocity)
            {
                for (int i = 0; i < kNumJoints; ++i)
                {
                    (*velocity)[i] = data->actual_joint_velocity[i];
                }
            }

            return true;
        }
    }

    auto *pose = drfl_.get_current_posj();
    if (!pose)
    {
        pose = drfl_.get_current_pose(ROBOT_SPACE_JOINT);
    }
    if (!pose)
    {
        return false;
    }

    if (position)
    {
        for (int i = 0; i < kNumJoints; ++i)
        {
            (*position)[i] = pose->_fPosition[i];
        }
    }

    if (velocity)
    {
        auto *current_velocity = drfl_.get_current_velj();
        if (current_velocity)
        {
            for (int i = 0; i < kNumJoints; ++i)
            {
                (*velocity)[i] = current_velocity->_fVelocity[i];
            }
        }
        else
        {
            velocity->fill(0.0f);
        }
    }

    return true;
}

bool DoosanController::holdCurrentPosition()
{
    JointArray current_position = {};
    JointArray current_velocity = {};

    for (int attempt = 0; attempt < 50; ++attempt)
    {
        if (readCurrentJointState(&current_position, &current_velocity))
        {
            setJointPositionTarget(current_position, JointArray{}, JointArray{}, config_.command_time_sec);
            std::cout << "[robot] holding current joint position: ";
            for (int i = 0; i < kNumJoints; ++i)
            {
                std::cout << current_position[i] << (i + 1 < kNumJoints ? ", " : "\n");
            }
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return false;
}

bool DoosanController::setJointPositionTrajectory(
    const JointArray &position,
    const JointArray &final_velocity,
    const JointArray &final_acceleration,
    float move_time_sec,
    float command_time_sec)
{
    JointArray start_position = {};
    JointArray start_velocity = {};
    bool got_state = false;
    for (int attempt = 0; attempt < 100; ++attempt)
    {
        if (readCurrentJointState(&start_position, &start_velocity))
        {
            got_state = true;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!got_state)
    {
        std::cerr << "[robot] failed to read current joint state for position trajectory\n";
        return false;
    }

    const float duration = std::max(
        move_time_sec > 0.0f ? move_time_sec : config_.position_move_time_sec,
        std::max(config_.rt_period_sec, 0.001f));
    const float tt = commandTimeOrDefault(command_time_sec, config_.command_time_sec);

    Command next_command;
    next_command.mode = CommandMode::JointPosition;
    next_command.command_time_sec = tt;
    next_command.trajectory_active = true;
    next_command.stop_rt_when_done = true;
    next_command.trajectory_start = Clock::now();
    next_command.trajectory_duration_sec = duration;
    next_command.position = start_position;
    next_command.velocity = final_velocity;
    next_command.acceleration = final_acceleration;
    next_command.target_position = position;

    const double tf = duration;
    const double tf2 = tf * tf;
    const double tf3 = tf2 * tf;
    const double tf4 = tf3 * tf;
    const double tf5 = tf4 * tf;

    for (int i = 0; i < kNumJoints; ++i)
    {
        const double ps = start_position[i];
        const double vs = start_velocity[i];
        const double as = 0.0;
        const double pf = position[i];
        const double vf = final_velocity[i];
        const double af = final_acceleration[i];

        next_command.a0[i] = static_cast<float>(ps);
        next_command.a1[i] = static_cast<float>(vs);
        next_command.a2[i] = static_cast<float>(as / 2.0);
        next_command.a3[i] = static_cast<float>(
            (20.0 * pf - 20.0 * ps - (8.0 * vf + 12.0 * vs) * tf -
             (3.0 * as - af) * tf2) /
            (2.0 * tf3));
        next_command.a4[i] = static_cast<float>(
            (30.0 * ps - 30.0 * pf + (14.0 * vf + 16.0 * vs) * tf +
             (3.0 * as - 2.0 * af) * tf2) /
            (2.0 * tf4));
        next_command.a5[i] = static_cast<float>(
            (12.0 * pf - 12.0 * ps - (6.0 * vf + 6.0 * vs) * tf -
             (as - af) * tf2) /
            (2.0 * tf5));
    }

    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_ = next_command;
    }

    std::cout << "[robot] position trajectory target: ";
    for (int i = 0; i < kNumJoints; ++i)
    {
        std::cout << position[i] << (i + 1 < kNumJoints ? ", " : "");
    }
    std::cout << " duration=" << duration << "s tt=" << tt << "s\n";
    return true;
}

bool DoosanController::setJointVelocityPositionTarget(
    const JointArray &target_position,
    const JointArray &max_velocity,
    const JointArray &acceleration,
    float gain,
    float tolerance_deg,
    float timeout_sec,
    float command_time_sec)
{
    JointArray current_position = {};
    bool got_state = false;
    for (int attempt = 0; attempt < 100; ++attempt)
    {
        if (readCurrentJointState(&current_position, nullptr))
        {
            got_state = true;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (!got_state)
    {
        std::cerr << "[robot] failed to read current joint state for velocity target control\n";
        return false;
    }

    double initial_max_error = 0.0;
    JointArray profile_direction = {};
    JointArray profile_peak_velocity = {};
    JointArray profile_accel_time = {};
    JointArray profile_cruise_time = {};
    JointArray profile_total_time = {};
    double profile_duration = 0.0;
    const double tolerance = static_cast<double>(tolerance_deg > 0.0f ? tolerance_deg : 0.5f);

    for (int i = 0; i < kNumJoints; ++i)
    {
        const double error =
            static_cast<double>(target_position[i]) -
            static_cast<double>(current_position[i]);
        const double distance = std::fabs(error);
        initial_max_error = std::max(initial_max_error, distance);

        if (distance <= tolerance)
        {
            continue;
        }

        const double max_velocity_abs = std::max(
            static_cast<double>(
                std::fabs(max_velocity[i]) > 0.0f
                    ? std::fabs(max_velocity[i])
                    : std::fabs(config_.rt_velocity_limit[i])),
            1.0e-6);
        const double acceleration_abs = std::max(
            static_cast<double>(
                std::fabs(acceleration[i]) > 0.0f
                    ? std::fabs(acceleration[i])
                    : std::fabs(config_.rt_acceleration_limit[i])),
            1.0e-6);

        const double accel_time_to_max = max_velocity_abs / acceleration_abs;
        const double accel_distance_to_max =
            0.5 * acceleration_abs * accel_time_to_max * accel_time_to_max;

        double accel_time = accel_time_to_max;
        double cruise_time = 0.0;
        double peak_velocity = max_velocity_abs;

        if (distance <= 2.0 * accel_distance_to_max)
        {
            peak_velocity = std::sqrt(distance * acceleration_abs);
            accel_time = peak_velocity / acceleration_abs;
        }
        else
        {
            const double cruise_distance = distance - 2.0 * accel_distance_to_max;
            cruise_time = cruise_distance / peak_velocity;
        }

        profile_direction[i] = static_cast<float>(error >= 0.0 ? 1.0 : -1.0);
        profile_peak_velocity[i] = static_cast<float>(peak_velocity);
        profile_accel_time[i] = static_cast<float>(accel_time);
        profile_cruise_time[i] = static_cast<float>(cruise_time);
        profile_total_time[i] = static_cast<float>(2.0 * accel_time + cruise_time);
        profile_duration = std::max(profile_duration, 2.0 * accel_time + cruise_time);
    }

    if (profile_duration > 0.0)
    {
        for (int i = 0; i < kNumJoints; ++i)
        {
            if (profile_total_time[i] <= 0.0f)
            {
                continue;
            }

            const double distance = std::fabs(
                static_cast<double>(target_position[i]) -
                static_cast<double>(current_position[i]));
            const double max_velocity_abs = std::max(
                static_cast<double>(
                    std::fabs(max_velocity[i]) > 0.0f
                        ? std::fabs(max_velocity[i])
                        : std::fabs(config_.rt_velocity_limit[i])),
                1.0e-6);
            const double acceleration_abs = std::max(
                static_cast<double>(
                    std::fabs(acceleration[i]) > 0.0f
                        ? std::fabs(acceleration[i])
                        : std::fabs(config_.rt_acceleration_limit[i])),
                1.0e-6);

            const double discriminant = std::max(
                0.0,
                profile_duration * profile_duration -
                    4.0 * distance / acceleration_abs);
            double accel_time =
                0.5 * (profile_duration - std::sqrt(discriminant));
            double peak_velocity = acceleration_abs * accel_time;
            if (peak_velocity > max_velocity_abs)
            {
                peak_velocity = max_velocity_abs;
                accel_time = peak_velocity / acceleration_abs;
            }
            const double cruise_time = std::max(0.0, profile_duration - 2.0 * accel_time);

            profile_peak_velocity[i] = static_cast<float>(peak_velocity);
            profile_accel_time[i] = static_cast<float>(accel_time);
            profile_cruise_time[i] = static_cast<float>(cruise_time);
            profile_total_time[i] = static_cast<float>(profile_duration);
        }
    }

    Command next_command;
    next_command.mode = CommandMode::JointVelocityTarget;
    next_command.command_time_sec = commandTimeOrDefault(command_time_sec, config_.command_time_sec);
    next_command.trajectory_active = false;
    next_command.stop_rt_when_done = true;
    next_command.velocity_target_start = Clock::now();
    next_command.velocity_target_last_update = next_command.velocity_target_start;
    next_command.velocity_gain = gain > 0.0f ? gain : 1.0f;
    next_command.velocity_tolerance_deg = tolerance_deg > 0.0f ? tolerance_deg : 0.5f;
    next_command.velocity_timeout_sec = timeout_sec > 0.0f ? timeout_sec : 20.0f;
    next_command.position = current_position;
    next_command.velocity = max_velocity;
    next_command.acceleration = acceleration;
    next_command.velocity_target_command.fill(0.0f);
    next_command.velocity_profile_direction = profile_direction;
    next_command.velocity_profile_peak = profile_peak_velocity;
    next_command.velocity_profile_accel_time = profile_accel_time;
    next_command.velocity_profile_cruise_time = profile_cruise_time;
    next_command.velocity_profile_total_time = profile_total_time;
    next_command.velocity_profile_duration_sec = static_cast<float>(profile_duration);
    next_command.target_position = target_position;

    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_ = next_command;
    }

    std::cout << "[robot] velocity trapezoid profile target: ";
    for (int i = 0; i < kNumJoints; ++i)
    {
        std::cout << target_position[i] << (i + 1 < kNumJoints ? ", " : "");
    }
    std::cout << " tolerance=" << next_command.velocity_tolerance_deg
              << " profile_duration=" << next_command.velocity_profile_duration_sec
              << " timeout=" << next_command.velocity_timeout_sec
              << "s tt=" << next_command.command_time_sec << "s\n";
    std::cout << "[robot] velocity target start position: ";
    for (int i = 0; i < kNumJoints; ++i)
    {
        std::cout << current_position[i] << (i + 1 < kNumJoints ? ", " : "");
    }
    std::cout << " initial_max_error=" << initial_max_error << "deg\n";
    return true;
}

void DoosanController::realtimeLoop()
{
    const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(std::max(config_.rt_period_sec, 0.001f)));

    while (realtime_loop_running_ && !realtime_control_running_)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    auto next = std::chrono::steady_clock::now();

    while (realtime_loop_running_ && realtime_control_running_)
    {
        next += period;

        Command command_snapshot;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command_snapshot = command_;
        }

        if (command_snapshot.mode == CommandMode::JointPosition)
        {
            JointArray position = command_snapshot.position;
            JointArray velocity = command_snapshot.velocity;
            JointArray acceleration = command_snapshot.acceleration;
            bool trajectory_done = false;

            if (command_snapshot.trajectory_active)
            {
                const double elapsed_sec = std::chrono::duration<double>(
                    Clock::now() - command_snapshot.trajectory_start)
                                               .count();
                const double tf = std::max(
                    static_cast<double>(command_snapshot.trajectory_duration_sec),
                    static_cast<double>(config_.rt_period_sec));
                const double t = std::clamp(elapsed_sec, 0.0, tf);
                const double t2 = t * t;
                const double t3 = t2 * t;
                const double t4 = t3 * t;
                const double t5 = t4 * t;
                trajectory_done = elapsed_sec >= tf;

                for (int i = 0; i < kNumJoints; ++i)
                {
                    const double a0 = command_snapshot.a0[i];
                    const double a1 = command_snapshot.a1[i];
                    const double a2 = command_snapshot.a2[i];
                    const double a3 = command_snapshot.a3[i];
                    const double a4 = command_snapshot.a4[i];
                    const double a5 = command_snapshot.a5[i];

                    position[i] = static_cast<float>(
                        a0 + a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5);
                    velocity[i] = static_cast<float>(
                        a1 + 2.0 * a2 * t + 3.0 * a3 * t2 + 4.0 * a4 * t3 +
                        5.0 * a5 * t4);
                    acceleration[i] = static_cast<float>(
                        2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t2 + 20.0 * a5 * t3);

                    if (trajectory_done)
                    {
                        position[i] = command_snapshot.target_position[i];
                        velocity[i] = command_snapshot.velocity[i];
                        acceleration[i] = command_snapshot.acceleration[i];
                    }
                }
            }

            std::lock_guard<std::mutex> lock(drfl_mutex_);
            drfl_.servoj_rt(
                position.data(),
                velocity.data(),
                acceleration.data(),
                command_snapshot.command_time_sec);

            if (trajectory_done && command_snapshot.stop_rt_when_done)
            {
                std::lock_guard<std::mutex> command_lock(command_mutex_);
                command_.mode = CommandMode::Hold;
                command_.trajectory_active = false;
                command_.stop_rt_when_done = false;
                realtime_loop_running_ = false;
            }
        }
        else if (command_snapshot.mode == CommandMode::JointVelocity)
        {
            std::lock_guard<std::mutex> lock(drfl_mutex_);
            drfl_.speedj_rt(
                command_snapshot.velocity.data(),
                command_snapshot.acceleration.data(),
                command_snapshot.command_time_sec);
        }
        else if (command_snapshot.mode == CommandMode::JointVelocityTarget)
        {
            JointArray current_position = command_snapshot.position;
            JointArray velocity = {};
            JointArray acceleration = command_snapshot.acceleration;
            bool target_done = false;
            const auto now = Clock::now();
            const double dt = std::max(
                std::chrono::duration<double>(now - command_snapshot.velocity_target_last_update).count(),
                static_cast<double>(config_.rt_period_sec));
            const double elapsed_sec = std::chrono::duration<double>(
                now - command_snapshot.velocity_target_start)
                                           .count();
            double max_error = 0.0;

            if (!readCurrentJointState(&current_position, nullptr))
            {
                std::cerr << "[robot] failed to read current joint state during velocity target control\n";
                current_position = command_snapshot.position;
            }
            else
            {
                double max_feedback_step = 0.0;
                double max_allowed_step = 5.0;
                for (int i = 0; i < kNumJoints; ++i)
                {
                    max_feedback_step = std::max(
                        max_feedback_step,
                        std::fabs(
                            static_cast<double>(current_position[i]) -
                            static_cast<double>(command_snapshot.position[i])));
                    max_allowed_step = std::max(
                        max_allowed_step,
                        std::fabs(static_cast<double>(command_snapshot.velocity_profile_peak[i])) * dt + 5.0);
                }
                if (max_feedback_step > max_allowed_step)
                {
                    current_position = command_snapshot.position;
                }
            }

            for (int i = 0; i < kNumJoints; ++i)
            {
                const double error =
                    static_cast<double>(command_snapshot.target_position[i]) -
                    static_cast<double>(current_position[i]);
                max_error = std::max(max_error, std::fabs(error));

                const double total_time = command_snapshot.velocity_profile_total_time[i];
                const double accel_time = command_snapshot.velocity_profile_accel_time[i];
                const double cruise_time = command_snapshot.velocity_profile_cruise_time[i];
                const double peak_velocity = command_snapshot.velocity_profile_peak[i];
                const double direction = command_snapshot.velocity_profile_direction[i];

                double speed = 0.0;
                if (total_time > 0.0 && accel_time > 0.0 && elapsed_sec < total_time)
                {
                    if (elapsed_sec < accel_time)
                    {
                        speed = peak_velocity * (elapsed_sec / accel_time);
                    }
                    else if (elapsed_sec < accel_time + cruise_time)
                    {
                        speed = peak_velocity;
                    }
                    else
                    {
                        const double decel_elapsed = elapsed_sec - accel_time - cruise_time;
                        speed = peak_velocity * std::max(
                            0.0,
                            1.0 - decel_elapsed / accel_time);
                    }
                }

                velocity[i] = static_cast<float>(direction * speed);
            }

            const bool timed_out = elapsed_sec >= command_snapshot.velocity_timeout_sec;
            target_done =
                elapsed_sec >= static_cast<double>(command_snapshot.velocity_profile_duration_sec) ||
                timed_out;

            if (target_done)
            {
                std::cout << "[robot] velocity trapezoid profile done max_error="
                          << max_error << "deg elapsed=" << elapsed_sec
                          << "s" << (timed_out ? " timed_out" : "") << '\n';
                velocity.fill(0.0f);
            }

            {
                std::lock_guard<std::mutex> lock(drfl_mutex_);
                drfl_.speedj_rt(
                    velocity.data(),
                    acceleration.data(),
                    command_snapshot.command_time_sec);
            }

            if (target_done && command_snapshot.stop_rt_when_done)
            {
                std::lock_guard<std::mutex> command_lock(command_mutex_);
                command_.mode = CommandMode::Hold;
                command_.trajectory_active = false;
                command_.stop_rt_when_done = false;
                command_.velocity_target_command = velocity;
                command_.velocity_target_last_update = now;
                command_.position = current_position;
                realtime_loop_running_ = false;
            }
            else
            {
                std::lock_guard<std::mutex> command_lock(command_mutex_);
                if (command_.mode == CommandMode::JointVelocityTarget)
                {
                    command_.velocity_target_command = velocity;
                    command_.velocity_target_last_update = now;
                    command_.position = current_position;
                }
            }
        }

        std::this_thread::sleep_until(next);
    }

    stopRealtimeSession();
}

void DoosanController::handleAccessControl(MONITORING_ACCESS_CONTROL access)
{
    std::cout << "[callback] access control: " << accessControlToString(access) << '\n';

    if (access == MONITORING_ACCESS_CONTROL_GRANT)
    {
        has_control_access_ = true;
        is_standby_ = false;
    }
    else if (access == MONITORING_ACCESS_CONTROL_DENY ||
             access == MONITORING_ACCESS_CONTROL_LOSS)
    {
        has_control_access_ = false;
        is_standby_ = false;
    }
}

void DoosanController::handleRobotState(ROBOT_STATE state)
{
    std::cout << "[callback] robot state: " << robotStateToString(state) << '\n';
    is_standby_ = state == STATE_STANDBY;
}

void DoosanController::handleDisconnected()
{
    std::cerr << "[callback] disconnected from robot\n";
    disconnected_ = true;
    connected_ = false;
    servo_on_ = false;
    realtime_loop_running_ = false;
    realtime_control_running_ = false;
    realtime_control_connected_ = false;
}

void DoosanController::onAccessControl(MONITORING_ACCESS_CONTROL access)
{
    if (active_instance_)
    {
        active_instance_->handleAccessControl(access);
    }
}

void DoosanController::onRobotState(ROBOT_STATE state)
{
    if (active_instance_)
    {
        active_instance_->handleRobotState(state);
    }
}

void DoosanController::onDisconnected()
{
    if (active_instance_)
    {
        active_instance_->handleDisconnected();
    }
}

const char *DoosanController::robotStateToString(ROBOT_STATE state)
{
    switch (state)
    {
    case STATE_INITIALIZING: return "INITIALIZING";
    case STATE_STANDBY: return "STANDBY";
    case STATE_MOVING: return "MOVING";
    case STATE_SAFE_OFF: return "SAFE_OFF";
    case STATE_TEACHING: return "TEACHING";
    case STATE_SAFE_STOP: return "SAFE_STOP";
    case STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
    case STATE_HOMMING: return "HOMMING";
    case STATE_RECOVERY: return "RECOVERY";
    case STATE_SAFE_STOP2: return "SAFE_STOP2";
    case STATE_SAFE_OFF2: return "SAFE_OFF2";
    case STATE_NOT_READY: return "NOT_READY";
    default: return "UNKNOWN";
    }
}

const char *DoosanController::accessControlToString(MONITORING_ACCESS_CONTROL access)
{
    switch (access)
    {
    case MONITORING_ACCESS_CONTROL_REQUEST: return "REQUEST";
    case MONITORING_ACCESS_CONTROL_GRANT: return "GRANT";
    case MONITORING_ACCESS_CONTROL_DENY: return "DENY";
    case MONITORING_ACCESS_CONTROL_LOSS: return "LOSS";
    default: return "UNKNOWN";
    }
}

const char *DoosanController::robotSystemToString(RobotSystem system)
{
    return system == RobotSystem::Real ? "real" : "virtual";
}

}  // namespace doosan
