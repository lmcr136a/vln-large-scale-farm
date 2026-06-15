#pragma once

// Geodetic helpers and GPS-origin sidecar IO for the localizer.
// Kept free of GLIM/ROS types so it stays small and unit-testable.

#include <cmath>
#include <string>
#include <fstream>
#include <Eigen/Core>

namespace glim_localizer {

inline Eigen::Vector3d wgs84_to_ecef(double lat, double lon, double alt) {
  const double a = 6378137.0;
  const double f = 1.0 / 298.257223563;
  const double e2 = f * (2.0 - f);
  const double lat_r = lat * M_PI / 180.0;
  const double lon_r = lon * M_PI / 180.0;
  const double N = a / std::sqrt(1.0 - e2 * std::sin(lat_r) * std::sin(lat_r));
  return Eigen::Vector3d(
    (N + alt) * std::cos(lat_r) * std::cos(lon_r),
    (N + alt) * std::cos(lat_r) * std::sin(lon_r),
    (N * (1.0 - e2) + alt) * std::sin(lat_r));
}

// ENU rotation at the given origin lat/lon (rows = East, North, Up axes).
inline Eigen::Matrix3d enu_rotation(double lat, double lon) {
  const double lat_r = lat * M_PI / 180.0;
  const double lon_r = lon * M_PI / 180.0;
  Eigen::Matrix3d R;
  R.row(0) = Eigen::Vector3d(-std::sin(lon_r), std::cos(lon_r), 0.0);
  R.row(1) = Eigen::Vector3d(-std::sin(lat_r) * std::cos(lon_r),
                             -std::sin(lat_r) * std::sin(lon_r), std::cos(lat_r));
  R.row(2) = Eigen::Vector3d(std::cos(lat_r) * std::cos(lon_r),
                             std::cos(lat_r) * std::sin(lon_r), std::sin(lat_r));
  return R;
}

struct GpsOrigin {
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
};

// Minimal JSON writer: {"lat":..,"lon":..,"alt":..}
inline bool save_gps_origin(const std::string& path, const GpsOrigin& o) {
  std::ofstream f(path);
  if (!f) return false;
  f.setf(std::ios::fixed);
  f.precision(9);
  f << "{\"lat\":" << o.lat << ",\"lon\":" << o.lon
    << ",\"alt\":" << o.alt << "}\n";
  return true;
}

// Minimal JSON reader for the file written above. Tolerant of whitespace.
inline bool load_gps_origin(const std::string& path, GpsOrigin& o) {
  std::ifstream f(path);
  if (!f) return false;
  std::string s((std::istreambuf_iterator<char>(f)),
                std::istreambuf_iterator<char>());
  auto grab = [&](const std::string& key, double& out) -> bool {
    const std::string pat = "\"" + key + "\"";
    auto p = s.find(pat);
    if (p == std::string::npos) return false;
    p = s.find(':', p);
    if (p == std::string::npos) return false;
    try { out = std::stod(s.substr(p + 1)); } catch (...) { return false; }
    return true;
  };
  return grab("lat", o.lat) && grab("lon", o.lon) && grab("alt", o.alt);
}

}  // namespace glim_localizer