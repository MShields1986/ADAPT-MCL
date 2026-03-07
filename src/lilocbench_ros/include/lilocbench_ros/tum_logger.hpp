#pragma once

#include <fstream>
#include <string>

namespace lilocbench_ros {

/// Accumulates poses and writes them in TUM format on flush/destruction.
/// TUM line: "timestamp x y z qx qy qz qw\n" (space-separated, Unix seconds)
/// For 2D: z=0, qx=qy=0, qz=sin(theta/2), qw=cos(theta/2).
class TumLogger {
 public:
  explicit TumLogger(const std::string& filepath);
  ~TumLogger();

  /// Append one pose.
  /// @param unix_stamp  Unix timestamp in seconds (float64).
  /// @param x, y, theta  2D pose in metres and radians.
  void log(double unix_stamp, float x, float y, float theta);

  /// Flush the buffer to disk. Called automatically on destruction.
  void flush();

  /// Create symlinks run_2.txt and run_3.txt pointing at run_1.txt.
  /// Call after flush() once run_1.txt has been written.
  static void make_run_symlinks(const std::string& run1_filepath);

 private:
  std::string filepath_;
  std::ofstream file_;
};

}  // namespace lilocbench_ros
