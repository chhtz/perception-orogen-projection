/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef PROJECTION_COLORIZEPOINTCLOUD_TASK_HPP
#define PROJECTION_COLORIZEPOINTCLOUD_TASK_HPP

#include "projection/ColorizePointcloudBase.hpp"
#include <frame_helper/FrameHelper.h>

namespace projection {

    /*! \class ColorizePointcloud 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','projection::ColorizePointcloud')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class ColorizePointcloud : public ColorizePointcloudBase
    {
	friend class ColorizePointcloudBase;
    protected:

        bool hasImage;
	frame_helper::FrameHelper frameHelper;
	base::samples::Pointcloud points;
	base::samples::frame::Frame frame;

        virtual void cameraCallback(const base::Time &ts, const ::RTT::extras::ReadOnlyPointer< ::base::samples::frame::Frame > &camera_sample);

        virtual void pointsCallback(const base::Time &ts, const ::base::samples::Pointcloud &points_sample);

        void colorizePointCloud(base::samples::Pointcloud& pointsCloud, base::samples::frame::Frame& image, const Eigen::Matrix4d& points2Cam);
        
    public:
        /** TaskContext constructor for ColorizePointcloud
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        ColorizePointcloud(std::string const& name = "projection::ColorizePointcloud");

        /** TaskContext constructor for ColorizePointcloud 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        ColorizePointcloud(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of ColorizePointcloud
         */
	~ColorizePointcloud();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif

