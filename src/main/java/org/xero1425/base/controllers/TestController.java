package org.xero1425.base.controllers;

import org.xero1425.base.XeroRobot;

/// \file

/// \brief A controller for the test mode.  This is not yet implemented in the XeroFramework
public class TestController extends BaseController
{
    /// \brief Create the test controller
    /// \param robot the robot1` object
    /// \param name the name of the controller
    public TestController(XeroRobot robot, String name) {
        super(robot, name) ;
    }

    /// \brief called to initialize the robot in test mode,
    @Override
    public void init() {
    }

    /// \brief called in each robot loop to control the robot in test mode
    @Override
    public void run() {        
    }
} ;