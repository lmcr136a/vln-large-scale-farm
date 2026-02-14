#ifndef GNSS_PROCESSING_HPP
#define GNSS_PROCESSING_HPP

#include <Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>

// Type definitions for compatibility
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;

class GnssProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    double time;
    double latitude;
    double longitude;
    double altitude;
    double local_E;
    double local_N;
    double local_U;
    int status;
    int service;

    double origin_longitude;
    double origin_latitude;
    double origin_altitude;

    V3D pose_cov;

    GnssProcess();
    ~GnssProcess();

    void InitOriginPosition(double latitude, double longitude, double altitude);
    void UpdateXYZ(double latitude, double longitude, double altitude);

    void Reverse(
        const double &local_E, const double &local_N, const double &local_U,
        double &lat, double &lon, double &alt
    );

    void set_extrinsic(const V3D &transl, const M3D &rot);
    void set_extrinsic(const V3D &transl);
    void set_extrinsic(const Eigen::Matrix4d &T);

private:
    GeographicLib::LocalCartesian geo_converter;

    M3D Gnss_R_wrt_Lidar;
    V3D Gnss_T_wrt_Lidar;
};

GnssProcess::GnssProcess()
{
    time = 0.0;
    local_E = 0.0;
    local_N = 0.0;
    local_U = 0.0;
    status = 0;
    service = 0;

    origin_longitude = 0;
    origin_latitude = 0;
    origin_altitude = 0;
    pose_cov = V3D::Zero();
    Gnss_T_wrt_Lidar = V3D::Zero();
    Gnss_R_wrt_Lidar = M3D::Identity();
}

GnssProcess::~GnssProcess() {}

// Initialize origin position: WGS84 -> ENU
void GnssProcess::InitOriginPosition(double latitude, double longitude, double altitude)
{
    geo_converter.Reset(latitude, longitude, altitude);
    origin_latitude = latitude;
    origin_longitude = longitude;
    origin_altitude = altitude;
}

// Get updated ENU coordinates
void GnssProcess::UpdateXYZ(double latitude, double longitude, double altitude) 
{
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

void GnssProcess::Reverse(
    const double &local_E, const double &local_N, const double &local_U,
    double &lat, double &lon, double &alt
) 
{
    geo_converter.Reverse(local_E, local_N, local_U, lat, lon, alt);
}

void GnssProcess::set_extrinsic(const Eigen::Matrix4d &T)
{
    Gnss_T_wrt_Lidar = T.block<3, 1>(0, 3);
    Gnss_R_wrt_Lidar = T.block<3, 3>(0, 0);
}

void GnssProcess::set_extrinsic(const V3D &transl)
{
    Gnss_T_wrt_Lidar = transl;
    Gnss_R_wrt_Lidar.setIdentity();
}

void GnssProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
    Gnss_T_wrt_Lidar = transl;
    Gnss_R_wrt_Lidar = rot;
}

#endif // GNSS_PROCESSING_HPP
