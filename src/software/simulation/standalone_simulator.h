#pragma once

#include "software/networking/threaded_nanopb_primitive_set_multicast_listener.h"
#include "software/networking/threaded_proto_multicast_listener.h"
#include "software/networking/threaded_proto_multicast_sender.h"
#include "software/parameter/dynamic_parameters.h"
#include "software/simulation/threaded_simulator.h"

extern "C"
{
#include "shared/proto/tbots_software_msgs.nanopb.h"
}

/**
 * This class abstracts all simulation and networking operations for
 * a StandaloneSimulator. The StandaloneSimulator can be run as a separate
 * application on a computer or network and be interacted with by up to
 * 2 instances of an AI.
 */
class StandaloneSimulator
{
   public:
    enum class SimulationMode
    {
        PLAY,
        PAUSE,
        SLOW_MOTION
    };

    /**
     * Creates a new StandaloneSimulator, and starts the simulation.
     *
     * @param standalone_simulator_config The config for the StandaloneSimulator
     */
    explicit StandaloneSimulator(
        std::shared_ptr<StandaloneSimulatorConfig> standalone_simulator_config);
    StandaloneSimulator() = delete;

    /**
     * Registers the given callback function. This callback function will be
     * called each time the simulation updates and a new SSL_WrapperPacket
     * is generated.
     *
     * @param callback The callback function to register
     */
    void registerOnSSLWrapperPacketReadyCallback(
        const std::function<void(SSL_WrapperPacket)>& callback);

    /**
     * Adds robots to predefined locations on the field
     */
    void setupInitialSimulationState();

    /**
     * Starts the simulation. If the simulator is already running, this
     * function does nothing.
     */
    void startSimulation();

    /**
     * Stops the simulation. If the simulator is already stopped, this
     * function does nothing.
     */
    void stopSimulation();

    /**
     * Sets the slow motion multiplier for the simulation. Larger values
     * cause the simulation to run in slow motion. For example, a value
     * of 2.0 causes the simulation to run 2x slower.
     *
     * @pre multiplier is >= 1.0
     *
     * @param multiplier The slow motion multiplier
     */
    void setSlowMotionMultiplier(double multiplier);

    /**
     * Resets the slow motion multiplier value to let the simulation
     * run in real-time speed.
     */
    void resetSlowMotionMultiplier();

    /**
     * Sets the state of the ball in the simulation. No more than 1 ball may exist
     * in the simulation at a time. If a ball does not already exist, a ball
     * is added with the given state. If a ball already exists, it's state is set to the
     * given state.
     *
     * @param state The new ball state
     */
    void setBallState(const BallState& state);

    // This is a somewhat arbitrary value that results in slow motion
    // simulation looking appropriately / usefully slow
    static constexpr double DEFAULT_SLOW_MOTION_MULTIPLIER = 8.0;

   private:
    /**
     * Sets the primitives being simulated by the robots on the respective team
     *
     * @param primitive_set_msg The set of primitives to run on the respective team
     */
    void setYellowRobotPrimitives(PrimitiveSetMsg primitive_set_msg);
    void setBlueRobotPrimitives(PrimitiveSetMsg primitive_set_msg);

    /**
     * A helper function that sets up all networking functionality with
     * the networking information in the StandlaoneSimulatorConfig
     */
    void initNetworking();

    std::shared_ptr<const StandaloneSimulatorConfig> standalone_simulator_config;
    std::unique_ptr<ThreadedNanoPbPrimitiveSetMulticastListener>
        yellow_team_primitive_listener, blue_team_primitive_listener;
    std::unique_ptr<ThreadedProtoMulticastSender<SSL_WrapperPacket>>
        wrapper_packet_sender;
    ThreadedSimulator simulator;
};