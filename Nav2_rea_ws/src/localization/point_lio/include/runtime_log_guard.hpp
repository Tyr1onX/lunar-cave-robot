#pragma once

#include <cstdio>
#include <filesystem>
#include <fstream>

inline void open_runtime_debug_logs(
  bool enabled, const std::filesystem::path & root_dir, FILE *& position_log,
  std::ofstream & matrix_log, std::ofstream & imu_log)
{
  if (!enabled) {
    return;
  }

  const auto log_dir = root_dir / "Log";
  position_log = std::fopen((log_dir / "pos_log.txt").c_str(), "w");
  matrix_log.open(log_dir / "mat_out.txt", std::ios::out);
  imu_log.open(log_dir / "imu_pbp.txt", std::ios::out);
}

inline void close_runtime_debug_logs(
  bool enabled, FILE *& position_log, std::ofstream & matrix_log,
  std::ofstream & imu_log)
{
  if (!enabled) {
    return;
  }

  if (position_log != nullptr) {
    std::fclose(position_log);
    position_log = nullptr;
  }
  matrix_log.close();
  imu_log.close();
}
