package org.xero1425.base.subsystems.motorsubsystem ;

import org.xero1425.base.actions.Action;

/// \file

/// \brief The base class for all actions that target the MotorSubsystem
public abstract class MotorAction extends Action
{
    // The subsystem of interest
    private final MotorSubsystem motor_subsystem_;

    /// \brief Create the MotorAction
    /// \param subsystem the subsystem for the action
    public MotorAction(final MotorSubsystem subsystem) {
        super(subsystem.getRobot().getMessageLogger());
        motor_subsystem_ = subsystem;
    }

    /// \brief Return the subsystem this action targets
    /// \returns the subsystem this action targets
    public MotorSubsystem getSubsystem() {
        return motor_subsystem_;
    }


}
