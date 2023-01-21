package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.base.XeroRobot;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDACtrl;
import org.xero1425.misc.TrapezoidalProfile;
import org.xero1425.misc.XeroMath;

/// \file

/// \brief This action moves the position of a subsystem controlled by a motor to the requested
/// position.  It does this be computed a TrapezoidalProfile as a plan for the motion and then using
/// a PIDACtrl folllower to follow the plan.  The PIDACtrl follower is a PID controller based on position
/// that has feed forward terms for both target velocity and acceleration.  This action supports goto actions
/// that can be "up" or "down".    "Up" is defined as the position increasing to get from its current position 
/// to its target position and "Down" is defined as the position decreasing to get from its current position 
/// to its target position.  Different PID parameters are read from the settings file depending on direction.  If
/// this action is being used on something like a turret, then the "up" and "down" parameters are really clockwise
/// and counter clockwise and will be the same.  If this action is being used on a lift or elevator, then the "up"
/// and "down" are truely up and down and will differ as in one direction gravity is with the motion and in the 
/// other direction gravity is against the motion.
///
///     "subsystems" : {
///         "NAME" : {
///             "goto": {
///                 "threshold": 2.5,
///                 "maxa": 45,
///                 "maxd": -45,
///                 "maxv": 45
///             },
///             "follower" : {
///                 "up" : {
///                     "kp" : DOUBLE,
///                     "ki" : DOUBLE,
///                     "kd" : DOUBLE,
///                     "kf" : DOUBLE,
///                     "min" : DOUBLE,
///                     "max" : DOUBLE,
///                     "imax" : DOUBLE
///                  },
///                 "down" : {
///                     "kp" : DOUBLE,
///                     "ki" : DOUBLE,
///                     "kd" : DOUBLE,
///                     "kf" : DOUBLE,
///                     "min" : DOUBLE,
///                     "max" : DOUBLE,
///                     "imax" : DOUBLE
///                  }
///              }
///          }
///     }
///
/// 
public class MotorEncoderGotoAction extends MotorAction {
    
    /// The difference between the current position and the target position below which we consider
    /// the goal being met.
    private double threshold_ ;

    // The target position
    private double target_ ;

    // The start time for the action
    private double start_time_ ;

    // The start position for the action
    private double start_position_ ;

    // The PID controller to follow the plan
    PIDACtrl ctrl_ ;

    // The TrapezoidalProfile that is the plan to follow
    TrapezoidalProfile profile_ ;

    // If true, add a hold action at the end of the goto action to hold the
    // subsystem in place.
    boolean addhold_ ;

    // The plot ID for plotting the motion
    int plot_id_ ;

    static int name_id_ = 0 ;

    // The columns to plot
    static final String [] plot_columns_ = { "time (sec)", "tpos (m)", "apos (m)", "tvel (m/s)", "avel (m/s)", "out (volts)" } ;

    /// \brief Create the action
    /// \param sub the MotorEncoderSubsystem subsystem for the action    
    /// \param target the target position
    /// \param addhold if true, add a hold action when the goto action is complete
    public MotorEncoderGotoAction(MotorEncoderSubsystem sub, double target, boolean addhold)
            throws Exception {
        super(sub) ;

        if (!(sub instanceof MotorEncoderSubsystem))
            throw new Exception("This subsystem is not a MotorEncoderSubsystem") ;
                    
        target_ = target ;
        addhold_ = addhold ;

        ISettingsSupplier settings = sub.getRobot().getSettingsSupplier() ;
        profile_ = new TrapezoidalProfile(settings, "subsystems:" + sub.getName() + ":goto") ;
        plot_id_ = sub.initPlot(sub.getName() + "-" + toString(plot_id_++)) ;
    }

    /// \brief Create the action
    /// \param sub the MotorEncoderSubsystem subsystem for the action
    /// \param target a string that names the settings value in the settings file that contains the target value 
    /// \param addhold if true, add a hold action when the goto action is complete    
    public MotorEncoderGotoAction(MotorEncoderSubsystem sub, String target, boolean addhold)
            throws Exception {
        super(sub) ;

        if (!(sub instanceof MotorEncoderSubsystem))
            throw new Exception("This subsystem is not a MotorEncoderSubsystem") ;

        target_ = getSubsystem().getSettingsValue(target).getDouble() ;
        addhold_ = addhold ;
        
        ISettingsSupplier settings = sub.getRobot().getSettingsSupplier() ;
        profile_ = new TrapezoidalProfile(settings, "subsystems:" + sub.getName() + ":goto") ;
        plot_id_ = sub.initPlot(sub.getName() + "-" + toString(0)) ;        
    }

    public void setTarget(double target) throws BadParameterTypeException, MissingParameterException {
        target_ = target ;
        setTarget() ;
    }

    /// \brief Start the action, computing the plan using the trapezoidal profile.  This method also
    /// initializes the PID controller based on whether or not the motion us "up" or "down".
    public void start() throws Exception {
        super.start() ;
        setTarget() ;
        getSubsystem().startPlot(plot_id_, plot_columns_) ;
    }

    /// \brief Called once per robot loop to adjust the motor power to follow the motion plan
    // given by the TrapezoidalProfile class.
    public void run() throws Exception {
        super.run() ;

        MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem() ;
        XeroRobot robot = sub.getRobot() ;

        double dt = robot.getDeltaTime() ;
        double elapsed = robot.getTime() - start_time_ ;
        double position = sub.getPosition() ;
        double traveled = normalizePosition(sub, position - start_position_) ;

        if (elapsed > profile_.getTotalTime())
        {
            setDone() ;
            sub.setPower(0.0) ;
            sub.endPlot(plot_id_);
        }
        else
        {
            double targetDist = profile_.getDistance(elapsed) ;
            double targetVel = profile_.getVelocity(elapsed) ;
            double targetAcc = profile_.getAccel(elapsed) ;
            double out = ctrl_.getOutput(targetAcc, targetVel, targetDist, traveled, dt) ;
            sub.setPower(out) ;

            Double[] data = new Double[plot_columns_.length] ;
            data[0] = elapsed ;
            data[1] = start_position_ + targetDist ;
            data[2] = position ;
            data[3] = targetVel ;
            data[4] = sub.getVelocity() ;
            data[5] = out ;
            sub.addPlotData(plot_id_, data);
        }
    }

    /// \brief Cancel the action and set the motor power to zero
    public void cancel() {
        super.cancel() ;
        getSubsystem().setPower(0.0) ;
        getSubsystem().endPlot(plot_id_);
    }

    /// \brief Returns a human readable string describing the action
    /// \returns a human readable string describing the action
    public String toString(int indent) {
        return prefix(indent) + "MotorEncoderGotoAction," + getSubsystem().getName() + "," + Double.toString(target_) ;
    }

    // Normalize the difference between the target and the action.  For non-angular motors, this is just the
    // difference.  For angular target, this is the normalize difference in angles.
    private double normalizePosition(MotorEncoderSubsystem me, double pos) {
        if (me.isAngular())
            return XeroMath.normalizeAngleDegrees(pos) ;
        
        return pos ;
    }

    private void setTarget() throws BadParameterTypeException, MissingParameterException {
        MotorEncoderSubsystem sub = (MotorEncoderSubsystem)getSubsystem() ;

        // If addhold_ is true, set the default action that will be run when this action
        // is complete to a hold action to hold the end position
        if (addhold_)
            sub.setDefaultAction(new MotorEncoderHoldAction(sub, target_)) ;
        else
            sub.setDefaultAction(null) ;

        // Check the current position to see if we are done
        double dist = normalizePosition(sub, target_ - sub.getPosition()) ;
        if (Math.abs(dist) < threshold_)
        {
            setDone() ;
        }
        else
        {
            //
            // Initialize the follower
            //
            String config = "subsystems:" + sub.getName() + ":follower" ;
            ISettingsSupplier settings = sub.getRobot().getSettingsSupplier() ;
            if (dist < 0)
                ctrl_ = new PIDACtrl(settings, config + ":down", sub.isAngular());
            else
                ctrl_ = new PIDACtrl(settings, config + ":up", sub.isAngular()) ;

            // Update the trapezoidal profile based on when we are starting.
            profile_.update(dist, 0, 0) ;
            start_time_ = sub.getRobot().getTime() ;
            start_position_ = sub.getPosition() ;
        }
    }

}
