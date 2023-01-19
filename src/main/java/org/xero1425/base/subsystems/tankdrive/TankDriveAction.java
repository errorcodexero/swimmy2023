package org.xero1425.base.subsystems.tankdrive ;

/// \file

import org.xero1425.base.actions.Action;

/// \brief This class is the base class for all TankDriveSubsystem actions
public abstract class TankDriveAction extends Action
{
    /// \brief Create the object holding a reference to the subsystem
    /// \param drive the tankdrive subsystem
    public TankDriveAction(TankDriveSubsystem drive) {
        super(drive.getRobot().getMessageLogger()) ;
        tankdrive_ = drive ;
    }

    /// \brief return the tank drive subsystem
    /// returns the tank drive subsystem
    public TankDriveSubsystem getSubsystem() {
        return tankdrive_ ;
    }

    private TankDriveSubsystem tankdrive_ ;
}