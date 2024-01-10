#How To Write a Controller#

Review: The hardware_interface reads data from all the joints, storing it in state arrays like TalonStateInterface and JointStateInterface. Then, the controllers (loaded by the controller_manager) run an update, writing values to command arrays like TalonCommandInterface and PositionJointInterface. Finally, the hardware_interface writes the data from those command arrays to the actual hardware. This happens in a loop as long as the frcrobot_hardware_interface node is running.

How to actually write a controller for a piece of hardware:

1. Create a package with the type of controller -1. for example, "elevator controller" -1. and modify the CMakeLists.txt and package.xml so that the package is importing everything it needs.

2. In the include/controller_name directory of the package write your header file. Here's an example of a header file with everything you need to start out.
<code cpp>
namespace mech_controller *create a namespace for your package
{
*Create a class for the controller, a derived class from the controller_interface::MultiInterfaceController
*This just means that you can import different kinds of interfaces
*For example, you could modify both a Talon and a solenoid in the same controller
class MechController : public controller_interface::MultiInterfaceController
     <hardware_interface::TalonCommandInterface, hardware_interface::JointStateInterface> 
{

     public:
          *Constructor
          MechController() {} 
     
          /***These are the forward declarations for the functions that the controller_manager calls directly***/
          *Pass in these args exactly, otherwise the controller_manager won't correctly execute the function
          
          *This is called when the controller is loaded.
          *The first thing that's passed in is a pointer to the Robot hardware. 
          *From there, you can load the individual interfaces that you need to write to, like the TalonCommandInterface
          virtual bool init(hardware_interface::RobotHW *hw, 
                              ros::NodeHandle           &root_nh,
                              ros::NodeHandle           &controller_nh);
          *Called when the controller is started. Use these exact arguments.
          virtual void starting(const ros::Time &time);
          *Called repeatedly within the hardware loop. Use this to read from and write to joints
          virtual void update(const ros::Time &time, const ros::Duration &period);
          *Called when the controller is unloaded. Not used much in our code but needs to be included.
          virtual void stopping(const ros::Time &time);
          
     private:
          *This is a Talon controller interface. 
          *It wraps a TalonCommandHandle, which is a a pointer to the TalonCommandInterface
          *Use mech_joint_.setCommand() to write to the actual Talon.
         talon_controllers::TalonPercentOutputControllerInterface mech_joint_;
         *This is a non-Talon position joint interface. It can be used to control a digital output, like shifting gears
         hardware_interface::JointHandle shift_joint_;
         *This holds the next commands for the two joints that were declared above
         realtime_tools::RealtimeBuffer<double> mech_command_;
         realtime_tools::RealtimeBuffer<bool> shift_command_;
}; *end of class
} *end of namespace
     
```