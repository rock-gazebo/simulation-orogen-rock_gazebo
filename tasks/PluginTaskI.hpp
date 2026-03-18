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
     *
     * The tasks are instanciated and have their setGazebo method called in
     * the Configure step of the gazebo lifecycle. They are then triggered synchronously
     * in the PreUpdate step. Normal RTT lifecycle is handled externally by the
     * system's manager (e.g. Syskit)
     */
    struct PluginTaskI {
        virtual ~PluginTaskI()
        {
        }

        virtual void setSimTime(base::Time const& time) = 0;

        /** Hook called in the Configure step to let the task get information
         * about the Gazebo setup. It is called before configureHook/startHook
         */
        virtual void setGazebo(gz::sim::Entity const& entity,
            sdf::ElementConstPtr const& sdf,
            gz::sim::EntityComponentManager& ecm,
            gz::sim::EventManager& event_manager) = 0;

        /** Let other threads process gazebo-critical parts of their code
         *
         * This is called by the plugin at each update step, to allow for other
         * threads (e.g. the configureHook) to do actions that modify the gazebo
         * state.
         *
         * @see GazeboSync
         */
        virtual void gazeboCriticalZone() = 0;
    };
}

#endif
