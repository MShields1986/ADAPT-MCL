#include "lilocbench_ros/tum_logger.hpp"

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <stdexcept>

namespace lilocbench_ros {

TumLogger::TumLogger(const std::string& filepath) : filepath_(filepath) {
  std::filesystem::path p(filepath);
  std::filesystem::create_directories(p.parent_path());
  file_.open(filepath, std::ios::out | std::ios::trunc);
  if (!file_.is_open()) {
    throw std::runtime_error("TumLogger: cannot open " + filepath);
  }
  file_ << std::fixed << std::setprecision(6);
}

TumLogger::~TumLogger() { flush(); }

void TumLogger::log(double unix_stamp, float x, float y, float theta) {
  if (!file_.is_open()) return;
  float qz = std::sin(theta * 0.5f);
  float qw = std::cos(theta * 0.5f);
  file_ << unix_stamp << ' '
        << x << ' ' << y << " 0.0 "
        << "0.0 0.0 " << qz << ' ' << qw << '\n';
}

void TumLogger::flush() {
  if (file_.is_open()) {
    file_.flush();
    file_.close();
  }
}

void TumLogger::make_run_symlinks(const std::string& run1_filepath) {
  namespace fs = std::filesystem;
  fs::path run1(run1_filepath);
  fs::path dir = run1.parent_path();

  for (int i = 2; i <= 3; ++i) {
    fs::path link = dir / ("run_" + std::to_string(i) + ".txt");
    if (fs::exists(link) || fs::is_symlink(link)) {
      fs::remove(link);
    }
    fs::create_symlink(run1.filename(), link);
  }
}

}  // namespace lilocbench_ros
