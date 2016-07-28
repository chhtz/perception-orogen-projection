/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "DepthMapToImage.hpp"

#include <base-logging/Logging.hpp>

#include <opencv2/imgproc.hpp>
#include "frame_helper/FrameHelper.h"


using namespace projection;

DepthMapToImage::DepthMapToImage(std::string const& name)
    : DepthMapToImageBase(name)
{
}

DepthMapToImage::DepthMapToImage(std::string const& name, RTT::ExecutionEngine* engine)
    : DepthMapToImageBase(name, engine)
{
}

DepthMapToImage::~DepthMapToImage()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See DepthMapToImage.hpp for more detailed
// documentation about them.

bool DepthMapToImage::configureHook()
{
    if (! DepthMapToImageBase::configureHook())
        return false;
    return true;
}
bool DepthMapToImage::startHook()
{
    if (! DepthMapToImageBase::startHook())
        return false;
    return true;
}
void DepthMapToImage::updateHook()
{
    DepthMapToImageBase::updateHook();
    base::samples::DepthMap dm;
    if(_depth_map.read(dm) != RTT::NewData)
    {
        LOG_WARN_S << "Failed to read depthmap";
        return;
    }
    if(!_image.connected()) return; // no need to work, if nobody wants our output

    typedef std::vector<double> vecD;
    typedef std::vector<float> vecF;

    const vecD &horz = dm.horizontal_interval, &vert = dm.vertical_interval;
    const vecF &dist = dm.distances, &rem = dm.remissions;

//    std::cout << "horz: " << horz.front() << ", " << horz.back() << " ~~ " << horz.size() << "\n";

    cv::Size_<double> size;

    double x0 = horz.back(), &width = size.width = x0 - horz.front();
    if(width < 0.01) width += 2*M_PI;
    double y0 = vert.front(), &height = size.height = vert.back() - y0;
    if(height < 0.0) height += 2*M_PI;

    double v_res = vert.size() == 2 ? height / dm.vertical_size : 0;
    double h_res = horz.size() == 2 ? width  / dm.horizontal_size : 0;

    const double cycle = _depth_color_cycle.value(), res = _angular_resolution.get(), siz2 = _point_size.get() / 2;
    const bool color = (cycle > 0.0) && false; // TODO
    const bool useRem = rem.size() == dist.size();

    assert(dist.size() == dm.horizontal_size * dm.vertical_size);

    cv::Mat image = cv::Mat::zeros(size * res, color ? CV_8UC3 : CV_8UC1);

    vecF::const_iterator dit = dist.begin(), rit = rem.begin();

    cv::Point2i p1, p2;
    for(size_t y=0; y< dm.vertical_size; ++y)
    {
        double y_ang = v_res ? y*v_res : vert[y] - y0;
        if(y_ang < 0) y_ang += 2*M_PI;
        p1.y = y_ang * res - siz2;
        p2.y = y_ang * res + siz2;
        for(size_t x=0; x< dm.horizontal_size; ++x)
        {
            double x_ang = h_res ? (dm.horizontal_size - 1 - x) * h_res : x0 - horz[x];
            if(x_ang < 0) x_ang += 2*M_PI;
            p1.x = x_ang * res - siz2;
            p2.x = x_ang * res + siz2;

            cv::Scalar color(255, 255, 255, 255);

            if(false)
            {
                // TODO
            }
            if(useRem)
            {
                color *= *rit;
                ++rit;
            }
            cv::rectangle(image, p1, p2,color, -1);
        }
    }

    base::samples::frame::Frame *frame = new base::samples::frame::Frame;

    frame_helper::FrameHelper::copyMatToFrame(image, *frame);
    frame->time = dm.time;
    _image.write(frame);

}
void DepthMapToImage::errorHook()
{
    DepthMapToImageBase::errorHook();
}
void DepthMapToImage::stopHook()
{
    DepthMapToImageBase::stopHook();
}
void DepthMapToImage::cleanupHook()
{
    DepthMapToImageBase::cleanupHook();
}
