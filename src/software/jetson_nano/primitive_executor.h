#pragma once
#include "proto/primitive.pb.h"
#include "software/geom/vector.h"
#include "software/world/world.h"

class PrimitiveExecutor
{
   public:
    /**
     * Constructor
     *
     * @param robot_constants Current robots' constants
     */
    PrimitiveExecutor(unsigned int robot_id, RobotConstants_t& robot_constants);

    /**
     * Start running a primitive
     *
     * @param robot_constants The robot constants
     * @param world_msg The primitive to start
     */
    void startPrimitive(const TbotsProto::World &world_msg, const TbotsProto::Primitive &primitive);

    /**
     * Steps the current primitive and returns a direct control primitive with the
     * target wheel velocities
     *
     * @param robot_state The current robot_state to step the primitive on
     * @returns DirectPerWheelControl The per-wheel direct control primitive msg
     */
    std::unique_ptr<TbotsProto::DirectControlPrimitive> stepPrimitive(const World &world, unsigned int robot_id);

   private:
    /*
     * Compute the next target linear velocity the robot should be at
     * assuming max acceleration.
     *
     * @param primitive The MovePrimitive to compute the linear velocity for
     * @param robot_state The RobotState of the robot we are planning the current
     * primitive for
     * @returns Vector The target linear velocity
     */
    Vector getTargetLinearVelocity(const TbotsProto::MovePrimitive &move_primitive, const RobotState& robot_state) const;

    /*
     * Compute the next target angular velocity the robot should be at
     * assuming max acceleration.
     *
     * @param primitive The MovePrimitive to compute the angular velocity for
     * @param robot_state The RobotState of the robot we are planning the current
     * primitive for
     * @returns AngularVelocity The target angular velocity
     */
    AngularVelocity getTargetAngularVelocity(const TbotsProto::MovePrimitive &move_primitive, const RobotState& robot_state) const;

    /*
     * The AutoKickOrChip settings from the move primitive need to get copied over
     * to the direct control primitive.
     *
     * TODO (#2340) Remove
     *
     * @param src The move primitive to copy from
     * @param dest The direct primitive to copy to
     */
    void copyAutoChipOrKick(const TbotsProto::MovePrimitive& src,
                            TbotsProto::DirectControlPrimitive* dest);

    unsigned int robot_id_;
    RobotConstants_t robot_constants_;

    TbotsProto::Primitive current_primitive_;
    World current_world_;
    Simulator ;
};
