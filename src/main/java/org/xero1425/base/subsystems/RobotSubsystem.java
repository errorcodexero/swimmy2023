package org.xero1425.base.subsystems;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.oi.OISubsystem;

/// \file

/// \brief This class is the top level subsystem for the robot.  It is spected to be
/// overridden by a top level robot subsystem class that is robot specific.
public class RobotSubsystem extends Subsystem
{
    // The OI for the robot
    private OISubsystem oi_ ;

    // The tankdrive for the robot
    private DriveBaseSubsystem db_ ;

    /// \brief Create the robot subsystem
    /// \param robot the robot object
    /// \param name the name of the subsystem
    public RobotSubsystem(XeroRobot robot, String name) throws Exception {
        super(robot, name) ;

        oi_ = null ;
        db_ = null ;
    }

    /// \brief Add a child subsystem to this subsystem.  If the child is a drivebase
    /// or an OI, it is remembered as these are special cases.  Otherwise the child is
    /// just added to the current subsystem like any other child.  Adding more than one
    /// drivebase or OI will cause and expection.
    /// \param child the child subsystem to add.
    public void addChild(final Subsystem child) throws Exception {
        super.addChild(child) ;

        if (child.isOI()) {
            if (oi_ != null)
                throw new Exception("multiple OI subsystems added to robot subsystem") ;

            oi_ = (OISubsystem)child ;
        }
        else if (child.isDB()) {
            if (db_ != null)
                throw new Exception("multiple drivebase subsystems added to robot subsystem") ;

            db_ = (DriveBaseSubsystem)child ;
        }
    }

    /// \brief Return the OI subsystem for the robot
    /// \returns the OI subsystem for the robot
    public OISubsystem getOI() {
        return oi_ ;
    }

    /// \brief Return the tank drive subsystem for the robot
    /// \returns the tankdrive subsystme for the robot
    public DriveBaseSubsystem getDB() {
        return db_ ;
    }
} ;