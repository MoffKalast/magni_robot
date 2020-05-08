#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
//#include <ignition/math/Pose3.hh>
//#include "gazebo/physics/physics.hh"
//#include "gazebo/common/common.hh"
//#include "gazebo/gazebo.hh"

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
public:
  WorldPluginTutorial() : WorldPlugin()
  {
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr /* _sdf */)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
        
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }


	// Option 2: Insert model from string via function call.
        // Insert a sphere model from string
     /*   sdf::SDF sphereSDF;
        sphereSDF.SetFromString(
           "<sdf version ='1.4'>\
              <model name ='sphere'>\
                <pose>1 0 0 0 0 0</pose>\
                <link name ='link'>\
                  <pose>0 0 .5 0 0 0</pose>\
                  <collision name ='collision'>\
                    <geometry>\
                      <sphere><radius>0.5</radius></sphere>\
                    </geometry>\
                  </collision>\
                  <visual name ='visual'>\
                    <geometry>\
                      <sphere><radius>0.5</radius></sphere>\
                    </geometry>\
                  </visual>\
                </link>\
              </model>\
            </sdf>");
        // Demonstrate using a custom model name.
        sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
        model->GetAttribute("name")->SetFromString("unique_sphere");
        _world->InsertModelSDF(sphereSDF);
*/
    ROS_WARN("TEST");
  }

};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
