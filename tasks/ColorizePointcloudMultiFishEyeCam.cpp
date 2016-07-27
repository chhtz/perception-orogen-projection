/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ColorizePointcloudMultiFishEyeCam.hpp"
#include <base/TimeMark.hpp>

using namespace projection;

ColorizePointcloudMultiFishEyeCam::ColorizePointcloudMultiFishEyeCam(std::string const& name)
    : ColorizePointcloudMultiFishEyeCamBase(name)
{
}

ColorizePointcloudMultiFishEyeCam::ColorizePointcloudMultiFishEyeCam(std::string const& name, RTT::ExecutionEngine* engine)
    : ColorizePointcloudMultiFishEyeCamBase(name, engine)
{
}

ColorizePointcloudMultiFishEyeCam::~ColorizePointcloudMultiFishEyeCam()
{
}

void ColorizePointcloudMultiFishEyeCam::camera1Callback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &camera1_sample)
{
    frames[0] = *camera1_sample;
    hasImage1 = true;
    std::cout << "New Image 0. dt = " << ts - image_times[0] << "\n";
    image_times[0] = ts;
}

void ColorizePointcloudMultiFishEyeCam::camera2Callback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &camera2_sample)
{
    frames[1] = *camera2_sample;
    hasImage2 = true;
    std::cout << "New Image 1. dt = " << ts - image_times[1] << "\n";
    image_times[1] = ts;
}

struct __attribute__((__packed__)) rgb
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};


void ColorizePointcloudMultiFishEyeCam::depthMapCallback(const base::Time &ts, const ::base::samples::DepthMap &depthMap_sample)
{
    if(!(hasImage1 && hasImage2))
    {
        std::cerr << "Missing images\n";
        return;
    }
    depthMap_sample.convertDepthMapToPointCloud(points.points);
    std::cout << "Number of points: " << points.points.size() << ", relative age of images: "
            << (ts - image_times[0]).toMicroseconds() << "us, " << (ts-image_times[1]).toMicroseconds() << "us\n";
    points.time = depthMap_sample.time;

    // passthrough for debugging purposes (having synced depthmaps and images)
    if(_depthMap_out.connected())
        _depthMap_out.write(depthMap_sample);

    if(_camera1_out.connected())
        _camera1_out.write(new base::samples::frame::Frame(frames[0]));
    if(_camera2_out.connected())
        _camera2_out.write(new base::samples::frame::Frame(frames[1]));

    Eigen::Affine3d poses[2];
    poses[0] = _pc2Cam1.get().toTransform();
    poses[1] = _pc2Cam2.get().toTransform();
    fisheye::CylinderCamCalibration cyl = _cylinderCalibration.get();
    if(cyl.width <=0) cyl.width = 1280;
    if(cyl.height<=0) cyl.height = 720;
    if(base::isUnset(cyl.fx)) cyl.fx = 330;
    if(base::isUnset(cyl.fy)) cyl.fy = cyl.fx;
    if(base::isUnset(cyl.cx)) cyl.cx = cyl.width * 0.5;
    if(base::isUnset(cyl.cy)) cyl.cy = cyl.height * 0.5;

    // prepare the target pointcloud
    //init pointcloud color with black (unknown)
    points.colors.clear();
    points.colors.resize( points.points.size(), base::Vector4d(1.0,0.0,0,0.5));

    base::TimeMark time("colorize-loop");
    // iterate through all the points
    for( size_t i = 0; i < points.points.size(); i++ )
    {
        const base::Vector3d &point = points.points[i];
        double color_mag = -1;
        for(int j=0; j<2; ++j)
        {
            Eigen::Vector3d pC = poses[j] * point;
            double r = std::sqrt(pC.x()*pC.x() + pC.z() * pC.z());
            double theta = std::atan2(pC.x(), pC.z()); // x = r*sin(theta), z = r*cos(theta)
            int u = theta * cyl.fx + cyl.cx;
            int v = pC.y() / r * cyl.fy + cyl.cy;
            if(0<=u && u < cyl.width && 0<=v && v<cyl.height)
            {
                const rgb& c = frames[j].at<rgb>(u,v);
                base::Vector4d color( c.b, c.g, c.r, 255.0 );
                double mag = color.head<3>().squaredNorm();
                if(mag > color_mag )
                {
                    points.colors[i] = color/255.0;
                    color_mag = mag;
                }
            }
        }

    }
    std::cout << time << std::endl;
    _colored_points.write(points); return;



//    colorizePointCloud(points, frame1, _pc2Cam1.get(), frame2, _pc2Cam2.get());
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ColorizePointcloudMultiFishEyeCam.hpp for more detailed
// documentation about them.

bool ColorizePointcloudMultiFishEyeCam::configureHook()
{
    if (! ColorizePointcloudMultiFishEyeCamBase::configureHook())
        return false;
    return true;
}
bool ColorizePointcloudMultiFishEyeCam::startHook()
{
    if (! ColorizePointcloudMultiFishEyeCamBase::startHook())
        return false;
    return true;
}
void ColorizePointcloudMultiFishEyeCam::updateHook()
{
    ColorizePointcloudMultiFishEyeCamBase::updateHook();
}
void ColorizePointcloudMultiFishEyeCam::errorHook()
{
    ColorizePointcloudMultiFishEyeCamBase::errorHook();
}
void ColorizePointcloudMultiFishEyeCam::stopHook()
{
    ColorizePointcloudMultiFishEyeCamBase::stopHook();
}
void ColorizePointcloudMultiFishEyeCam::cleanupHook()
{
    ColorizePointcloudMultiFishEyeCamBase::cleanupHook();
    hasImage1 = hasImage2 = false;
}
