package org.xero1425.base.actions;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

/// \file

/// \brief a DelayAction delays the execution of the action until the delay expired
public class DelayAction extends Action {

    // Timer
    private XeroTimer timer_ ;

    /// \brief create a new DelayAction
    /// \param robot the robot object
    /// \param delay the amount of the delay in seconds
    public DelayAction(XeroRobot robot, double delay) {
        super(robot.getMessageLogger());

        timer_ = new XeroTimer(robot, "delay-action", delay) ;
    }

    /// \brief create a new DelayAction
    /// \param robot the robot object
    /// \param delay a string specifying a delay value to look up in the settings file
    public DelayAction(XeroRobot robot, String delay) throws BadParameterTypeException, MissingParameterException {
        super(robot.getMessageLogger()) ;
        double delayval = robot.getSettingsSupplier().get(delay).getDouble() ;
        timer_ = new XeroTimer(robot, "delay-action", delayval) ;
    }

    /// \brief create a new DelayAction
    /// \param robot the robot object
    /// \param delay a string specifying a delay value to look up in the settings file
    public DelayAction(Subsystem sub, String delay) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger()) ;
        double delayval = sub.getSettingsValue(delay).getDouble() ;
        timer_ = new XeroTimer(sub.getRobot(), "delay-action", delayval) ;
    }

    /// \brief start the delay action
    @Override
    public void start() throws Exception {
        super.start() ;
        timer_.start() ;
    }

    /// \brief run the delay action.
    /// Called each robot loop.  When the delay time value has expired this method sets the
    /// done state of the action to true.
    @Override
    public void run() throws Exception {
        super.run() ;
        if (timer_.isExpired())
            setDone() ;
    }

    /// \brief cancel the current action
    @Override
    public void cancel() {
        super.cancel() ;
    }

    /// \brief generate a human readable string describing the action
    /// \returns a human readable string describing the action
    @Override
    public String toString(int indent) {
        return prefix(indent) + "DelayAction " + Double.toString(timer_.getDuration()) ;
    }
}