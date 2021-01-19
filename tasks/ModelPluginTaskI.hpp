#ifndef ROCK_GAZEBO_MODEL_PLUGIN_TASK_I_HPP
#define ROCK_GAZEBO_MODEL_PLUGIN_TASK_I_HPP

#include <gazebo/physics/PhysicsTypes.hh>
#include <string>

namespace rock_gazebo {
    /**
     * Abstract base class used by the rock_gazebo plugin to instanciate tasks
     * associated with model plugins
     */
    struct ModelPluginTaskI {
        virtual ~ModelPluginTaskI() {}

        virtual void setGazeboModel(
            std::string const& pluginName,
            gazebo::physics::ModelPtr model
        ) = 0;
    };
}

#endif