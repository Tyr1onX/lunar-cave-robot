#include "runtime_log_guard.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

namespace fs = std::filesystem;

namespace
{
void write_file(const fs::path & path, const std::string & contents)
{
  std::ofstream output(path, std::ios::binary);
  output << contents;
}

std::string read_file(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

bool fail(const std::string & reason)
{
  std::cerr << "runtime log guard regression failed: " << reason << '\n';
  return false;
}
}  // namespace

int main()
{
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const fs::path root = fs::temp_directory_path() /
    ("point-lio-log-guard-" + std::to_string(nonce));
  const fs::path log_dir = root / "Log";
  fs::create_directories(log_dir);

  const fs::path position_path = log_dir / "pos_log.txt";
  const fs::path matrix_path = log_dir / "mat_out.txt";
  const fs::path imu_path = log_dir / "imu_pbp.txt";
  write_file(position_path, "position sentinel\n");
  write_file(matrix_path, "matrix sentinel\n");
  write_file(imu_path, "imu sentinel\n");

  FILE * position_log = nullptr;
  std::ofstream matrix_log;
  std::ofstream imu_log;
  open_runtime_debug_logs(false, root, position_log, matrix_log, imu_log);
  close_runtime_debug_logs(false, position_log, matrix_log, imu_log);
  if (position_log != nullptr || matrix_log.is_open() || imu_log.is_open()) {
    fs::remove_all(root);
    return fail("disabled logging opened a file");
  }
  if (read_file(position_path) != "position sentinel\n" ||
    read_file(matrix_path) != "matrix sentinel\n" ||
    read_file(imu_path) != "imu sentinel\n")
  {
    fs::remove_all(root);
    return fail("disabled logging changed an existing log file");
  }

  open_runtime_debug_logs(true, root, position_log, matrix_log, imu_log);
  const bool all_open = position_log != nullptr && matrix_log.is_open() && imu_log.is_open();
  close_runtime_debug_logs(true, position_log, matrix_log, imu_log);
  if (!all_open || position_log != nullptr || matrix_log.is_open() || imu_log.is_open()) {
    fs::remove_all(root);
    return fail("enabled logging did not open and close all three files");
  }
  if (!read_file(position_path).empty() || !read_file(matrix_path).empty() ||
    !read_file(imu_path).empty())
  {
    fs::remove_all(root);
    return fail("enabled logging no longer opens files with the expected truncate mode");
  }

  fs::remove_all(root);
  std::cout << "runtime log guard regression passed\n";
  return 0;
}
