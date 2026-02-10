#ifndef ROCK_GAZEBO_MODEL_PLUGIN_TASK_I_HPP
#define ROCK_GAZEBO_MODEL_PLUGIN_TASK_I_HPP

#include <base/Time.hpp>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>
#include <string>

namespace rock_gazebo {
    /**
     * Abstract base class used by the rock_gazebo plugin to instanciate tasks
     * associated with model plugins
     */
    struct PluginTaskI {
        virtual ~PluginTaskI()
        {
        }

        virtual void setSimTime(base::Time const& time) = 0;

        virtual void setGazebo(std::string const& pluginName,
            gz::sim::Entity const& entity,
            sdf::ElementConstPtr const& sdf,
            gz::sim::EntityComponentManager& ecm,
            gz::sim::EventManager& event_manager) = 0;

        virtual void setGazeboPluginTaskName(std::string const& pluginTaskName) = 0;

        virtual void gazeboCriticalZone() = 0;
    };
}

#endif
