package org.xero1425.base.controllers;

import org.xero1425.base.XeroRobot;

/// \file

/// \brief This class is the base class for all controllers.  A controller is the class that
/// controls the robot during the robot loop.  There is a different controller for Autonomous
/// mode versus Teleop (Operator Control) mode.
public abstract class BaseController
{
    // The robot object
    private XeroRobot robot_ ;

    // The name of the controller
    private String name_ ;

    /// \brief Create a new controller
    /// \param robot the robot
    /// \param name the name of the controller
    public BaseController(XeroRobot robot, String name) {
        robot_ = robot ;
        name_ = name ;
    }

    /// \brief Return the name of the controller
    /// \returns the name of the controller
    public String getName() {
        return name_ ;
    }

    /// \brief Called when the specific mode associated with this controller is started.  This
    /// method gives the controller the chance to perform any mode specific initialization.
    /// This method must be overridden by a derived class.
    public abstract void init() ;

    /// \brief Called during each robot loop to perform any control function required.  This method
    /// must be overridden by a derived class. 
    public abstract void run() ;

    /// \brief Returns the robot object
    /// \returns the robot object
    public XeroRobot getRobot() {
        return robot_ ;
    }

} ;
