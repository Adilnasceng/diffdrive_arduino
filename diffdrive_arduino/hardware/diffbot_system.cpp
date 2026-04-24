// Copyright 2021 ros2_control Development Team
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

#include "diffdrive_arduino/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Zorunlu parametreleri güvenli şekilde parse et
  auto get_required = [&](const std::string & key) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end() || it->second.empty())
    {
      throw std::runtime_error("Missing required hardware parameter: '" + key + "'");
    }
    return it->second;
  };

  try
  {
    cfg_.left_wheel_name = get_required("left_wheel_name");
    cfg_.right_wheel_name = get_required("right_wheel_name");
    cfg_.loop_rate = ::std::stof(get_required("loop_rate"));
    cfg_.device = get_required("device");
    cfg_.baud_rate = ::std::stoi(get_required("baud_rate"));
    cfg_.timeout_ms = ::std::stoi(get_required("timeout_ms"));
    cfg_.left_enc_counts_per_rev = ::std::stoi(get_required("left_enc_counts_per_rev"));
    cfg_.right_enc_counts_per_rev = ::std::stoi(get_required("right_enc_counts_per_rev"));
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "Hardware parametreleri okunamadı: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Sayısal parametreleri doğrula
  if (cfg_.loop_rate <= 0.0f || cfg_.left_enc_counts_per_rev <= 0 ||
      cfg_.right_enc_counts_per_rev <= 0)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "Geçersiz parametre: loop_rate=%.2f, left_enc=%d, right_enc=%d (hepsi > 0 olmalı)",
      cfg_.loop_rate, cfg_.left_enc_counts_per_rev, cfg_.right_enc_counts_per_rev);
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  try
  {
    // Buzzer eşik değerini config'den oku
    auto rst_it = info_.hardware_parameters.find("reverse_speed_threshold");
    if (rst_it != info_.hardware_parameters.end() && !rst_it->second.empty())
    {
      cfg_.reverse_speed_threshold = ::std::stod(rst_it->second);
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
                  "Reverse speed threshold set to: %.3f", cfg_.reverse_speed_threshold);
    }
    else
    {
      cfg_.reverse_speed_threshold = -0.1; // Default değer
      RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"),
                  "reverse_speed_threshold not found in config, using default: %.3f", cfg_.reverse_speed_threshold);
    }

    // Geri gitme buzzer'ı kontrol parametresi
    auto erb_it = info_.hardware_parameters.find("enable_reverse_buzzer");
    if (erb_it != info_.hardware_parameters.end())
    {
      const auto & erb_val = erb_it->second;
      cfg_.enable_reverse_buzzer = (erb_val == "true" || erb_val == "True" || erb_val == "1");
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"),
                  "Reverse buzzer enabled: %s", cfg_.enable_reverse_buzzer ? "true" : "false");
    }
    else
    {
      cfg_.enable_reverse_buzzer = true; // Default aktif
    }

    // PID parametreleri — hepsi birlikte olmalı
    if (info_.hardware_parameters.count("pid_p") > 0)
    {
      cfg_.pid_p = ::std::stoi(get_required("pid_p"));
      cfg_.pid_d = ::std::stoi(get_required("pid_d"));
      cfg_.pid_i = ::std::stoi(get_required("pid_i"));
      cfg_.pid_o = ::std::stoi(get_required("pid_o"));
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "Opsiyonel parametreler okunamadı: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.left_enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.right_enc_counts_per_rev);

  // Buzzer durumunu başlat
  buzzer_reverse_active_ = false;
  buzzer_manual_active_ = false;
  buzzer_active_ = false;

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

::std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
{
  ::std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

::std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  ::std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  try
  {
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "Serial port açılamadı '%s': %s", cfg_.device.c_str(), e.what());
    return hardware_interface::CallbackReturn::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  if (comms_.connected())
  {
    // GÜVENLİK: disconnect öncesi motoru durdur
    // try-catch: LibSerial::NotOpen gibi exception'ları yakalar (USB çekilmesi vb.)
    // return false kontrolü: ReadTimeout sessiz başarısızlığını yakalar
    try
    {
      if (!comms_.set_motor_values(0, 0))
      {
        RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"),
                    "Cleanup: motor durdurma komutu zaman aşımına uğradı.");
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Cleanup sırasında motor durdurulamadı: %s", e.what());
    }
    comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }

  // İlk read() velocity spike'ını önlemek için işaretle
  first_read_ = true;

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");

  // GÜVENLİK: deactivate sırasında motoru durdur
  if (comms_.connected())
  {
    try
    {
      if (!comms_.set_motor_values(0, 0))
      {
        RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"),
                    "Deactivate: motor durdurma komutu zaman aşımına uğradı.");
      }
      if (buzzer_active_)
      {
        comms_.set_buzzer_state(false);
        buzzer_active_ = false;
        buzzer_reverse_active_ = false;
        buzzer_manual_active_ = false;
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Deactivate sırasında motor durdurulamadı: %s", e.what());
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  if (!comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc))
  {
    // Timeout veya bozuk cevap — eski encoder değerleri korunur, bu kareyi atla.
    // first_read_ = true: sonraki başarılı okumada sadece pozisyon başlatılır,
    // velocity hesaplanmaz — iki cycle'lık birikmiş delta sıçramasını önler.
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"),
                "Encoder okunamadı, bu kare atlandı.");
    first_read_ = true;
    return hardware_interface::return_type::OK;
  }

  double delta_seconds = period.seconds();
  if (delta_seconds <= 0.0)
  {
    return hardware_interface::return_type::OK;
  }

  // İlk çağrıda sadece pozisyonu başlat — velocity spike'ını önle
  if (first_read_)
  {
    wheel_l_.pos = wheel_l_.calc_enc_angle();
    wheel_l_.vel = 0.0;
    wheel_r_.pos = wheel_r_.calc_enc_angle();
    wheel_r_.vel = 0.0;
    first_read_ = false;
    return hardware_interface::return_type::OK;
  }

  double pos_prev = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

  // Arduino reset / encoder 0'lanma tespiti:
  // Fiziksel olarak imkânsız hız sıçraması → encoder teleportation bug.
  // Eşik: 10 rad/s — herhangi bir AGV tekerleği bu hıza ulaşamaz.
  constexpr double MAX_VEL_JUMP = 10.0;
  if (std::abs(wheel_l_.vel) > MAX_VEL_JUMP || std::abs(wheel_r_.vel) > MAX_VEL_JUMP)
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"),
                "Anormal hız sıçraması tespit edildi (L=%.1f R=%.1f rad/s) — "
                "Arduino reset mi? Encoder pozisyonu yeniden başlatılıyor.",
                wheel_l_.vel, wheel_r_.vel);
    wheel_l_.vel = 0.0;
    wheel_r_.vel = 0.0;
    first_read_ = true;  // Sonraki frame'de pozisyon yeniden başlatılır, spike olmaz
    return hardware_interface::return_type::OK;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // NaN koruması: ROS2 kontrolcüsünden geçersiz hız gelirse motoru durdur
  if (std::isnan(wheel_l_.cmd) || std::isnan(wheel_r_.cmd) ||
      std::isinf(wheel_l_.cmd) || std::isinf(wheel_r_.cmd))
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"),
                "Geçersiz motor komutu (NaN/Inf) — motorlar durduruldu. L=%.3f R=%.3f",
                wheel_l_.cmd, wheel_r_.cmd);
    if (!comms_.set_motor_values(0, 0))
    {
      RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"),
                  "NaN/Inf guard: motor durdurma komutu zaman aşımına uğradı.");
    }
    return hardware_interface::return_type::OK;
  }

  // lround: 0.8 tick → 1, static_cast kesme: 0.8 → 0 (deadband — robot hareket etmez)
  int motor_l_counts_per_loop = static_cast<int>(std::lround(wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate));
  int motor_r_counts_per_loop = static_cast<int>(std::lround(wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate));

  // Motor komutu her zaman önce gönderilir
  if (!comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop))
  {
    RCLCPP_WARN(rclcpp::get_logger("DiffDriveArduinoHardware"),
                "Motor komutu uygulanamadı (timeout/hata).");
  }

  // Buzzer kontrolü motor komutundan sonra — aynı döngüde iki serial mesaj
  // gitmesi gerekirse motor komutu gecikmiş olmaz
  if (cfg_.enable_reverse_buzzer)
  {
    check_reverse_condition();
  }
  return hardware_interface::return_type::OK;
}

void DiffDriveArduinoHardware::set_manual_buzzer(bool active)
{
  if (buzzer_manual_active_ != active)
  {
    buzzer_manual_active_ = active;
    update_buzzer_state();
    
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                "Manual buzzer set to: %s", active ? "ON" : "OFF");
  }
}

void DiffDriveArduinoHardware::enable_reverse_buzzer(bool enable)
{
  cfg_.enable_reverse_buzzer = enable;
  
  if (!enable)
  {
    // Geri gitme buzzer'ını deaktif et
    buzzer_reverse_active_ = false;
    update_buzzer_state();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
              "Reverse buzzer %s", enable ? "enabled" : "disabled");
}

bool DiffDriveArduinoHardware::is_buzzer_active() const
{
  return buzzer_active_;
}

void DiffDriveArduinoHardware::update_buzzer_state()
{
  bool should_be_active = (buzzer_reverse_active_ || buzzer_manual_active_);
  
  if (should_be_active != buzzer_active_)
  {
    buzzer_active_ = should_be_active;
    comms_.set_buzzer_state(buzzer_active_);
    
    if (buzzer_active_)
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), 
                  "Buzzer ON - Reverse: %s, Manual: %s", 
                  buzzer_reverse_active_ ? "ON" : "OFF",
                  buzzer_manual_active_ ? "ON" : "OFF");
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Buzzer OFF");
    }
  }
}

void DiffDriveArduinoHardware::check_reverse_condition()
{
  bool is_reversing = (wheel_l_.cmd < cfg_.reverse_speed_threshold && 
                      wheel_r_.cmd < cfg_.reverse_speed_threshold);
  
  if (buzzer_reverse_active_ != is_reversing)
  {
    buzzer_reverse_active_ = is_reversing;
    update_buzzer_state();
  }
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)