package org.xero1425.base.controllers;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.subsystems.oi.OISubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

/// \file

/// \brief This is the teleop controller for the robot.  There is NOT expected to be a
/// class derived from this one as the robot specific customizations for teleop are generally
/// contained in the OI subsystem.  Hoewver, there may be a derived class to perform any 
/// initialization required in the init() method.  Note, if the per mode initialization is
/// limited to the subsystems, see the Subsystem.init(LoopType) method.
public class TeleopController extends BaseController
{
    /// \brief Create a new telop controller
    /// \param robot the robot object
    /// \param name the name of the controller
    public TeleopController(XeroRobot robot, String name) {
        super(robot, name) ;
    }

    /// \brief Called to initialize the robot for teleop mode, does nothings
    /// for this class, but can be overridden in a derived class.
    @Override
    public void init() {
    }

    /// \brief Called once per robot loop to determine what the operator is requesting
    /// and to execute requested operations.
    @Override 
    public void run() {
        // Get the OI from the robot
        OISubsystem oi = getRobot().getRobotSubsystem().getOI() ;
        
        try {
            // Generate a set of actions based on the OI
            if (oi != null)
                oi.generateActions() ;
        }
        catch(InvalidActionRequest ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("Exception generating actions in teleop - ") ;
            logger.add(ex.getMessage()) ;
            logger.logStackTrace(ex.getStackTrace());
            logger.endMessage(); 
        }
    }
} ;
