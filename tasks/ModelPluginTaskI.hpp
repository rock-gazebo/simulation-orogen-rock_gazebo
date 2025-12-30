#ifndef ROCK_GAZEBO_MODEL_PLUGIN_TASK_I_HPP
#define ROCK_GAZEBO_MODEL_PLUGIN_TASK_I_HPP

#include <gz/sim/System.hh>
#include <string>

namespace gz_rock {
    /**
     * Abstract base class used by the gz_rock plugin to instanciate tasks
     * associated with model plugins
     */
    struct ModelPluginTaskI {
        virtual ~ModelPluginTaskI() {}

        virtual void setGazebo(
            std::string const& pluginName,
            gz::sim::Entity const& entity,
            std::shared_ptr<const sdf::Element> const& sdf,
            gz::sim::EntityComponentManager& ecm,
            gz::sim::EventManager& event_manager
        ) = 0;

        virtual void setGazeboPluginTaskName(
            std::string const& pluginTaskName
        ) = 0;
    };
}

#endif