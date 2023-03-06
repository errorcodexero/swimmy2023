package org.xero1425.base.subsystems.motorsubsystem;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;

/// \file

/// \brief This action holds the motor at its current position.  It has two modes.  In the
/// first mode, an explicit position target is given.  In the second mode, the position to hold
/// is read from the MotorEncoderSubsystem when the action is started.  This second mode is used to
/// hold a current position.  This action requires the settings for the PID controller in the
/// settings file.  Note, NAME is the name of the subsystem and will be different for each different
/// subsystem.
///
///     "subsystems" : {
///         "NAME" : {
///             "hold" : {
///                 "kp" : DOUBLE,
///                 "ki" : DOUBLE,
///                 "kd" : DOUBLE,
///                 "kf" : DOUBLE,
///                 "min" : DOUBLE,
///                 "max" : DOUBLE,
///                 "imax" : DOUBLE
///              }
///          }
///     }
///
public class MotorEncoderHoldAction extends MotorAction {

    /// If true, an explicit hold position target was provided in the constructuro
    private boolean has_explicit_target_ ;

    // The target for the motor encoder subsystem position
    private double target_ ;

    // The PID controller that provides the motor output
    private PIDCtrl pid_ ;

    /// \brief Create the action that holds the motor at the position found when the action is started.
    /// \param subsystem the MotorEncoderSubsystem for the action
    public MotorEncoderHoldAction(MotorEncoderSubsystem subsystem)
            throws MissingParameterException, BadParameterTypeException {
        super(subsystem);
        has_explicit_target_ = false;
        target_ = Double.NaN ;

        String subname = "subsystems:" + subsystem.getName() + ":hold" ;
        pid_ = new PIDCtrl(subsystem.getRobot().getSettingsSupplier(), subname, subsystem.isAngular()) ;        
    }

    /// \brief Create the action that holds the motor at an explicit position
    /// \param subsystem the MotorEncoderSubsystem for the action    
    /// \param target the target for the motor
    public MotorEncoderHoldAction(MotorEncoderSubsystem subsystem, double target)
            throws MissingParameterException, BadParameterTypeException {
        super(subsystem);
        has_explicit_target_ = true;
        target_ = target;

        String subname = "subsystems:" + subsystem.getName() + ":hold" ;
        pid_ = new PIDCtrl(subsystem.getRobot().getSettingsSupplier(), subname, subsystem.isAngular()) ;           
    }

    /// \brief Create the action that holds the motor at an explicit position
    /// \param subsystem the MotorEncoderSubsystem for the action    
    /// \param target a string giving the name of a setting from the settings file with the target    
    public MotorEncoderHoldAction(MotorEncoderSubsystem subsystem, String target)
            throws BadParameterTypeException, MissingParameterException {
        super(subsystem) ;
        has_explicit_target_ = true ;
        target_ = subsystem.getSettingsValue(target).getDouble() ;

        String subname = "subsystems:" + subsystem.getName() + ":hold" ;
        pid_ = new PIDCtrl(subsystem.getRobot().getSettingsSupplier(), subname, subsystem.isAngular()) ;           
    }
    
    /// \brief Create the action that holds the motor at an explicit position
    /// \param subsystem the MotorEncoderSubsystem for the action    
    /// \param target a string giving the name of a setting from the settings file with the target    
    public MotorEncoderHoldAction(MotorEncoderSubsystem subsystem, String pidname, String target)
            throws BadParameterTypeException, MissingParameterException {
        super(subsystem) ;
        has_explicit_target_ = true ;
        target_ = subsystem.getSettingsValue(target).getDouble() ;

        String subname = "subsystems:" + subsystem.getName() + ":" + pidname ;
        pid_ = new PIDCtrl(subsystem.getRobot().getSettingsSupplier(), subname, subsystem.isAngular()) ;           
    }    

    /// \brief Returns the current target for the action.  If this type of action is an implicit action
    /// and the action has not been started, then this value is not valid and will return NaN.
    /// \returns the target for the hold action.
    public double getTarget() {
        return target_ ;
    }

    /// \brief Start this action, reading the current motor subsystem position if the target is not explicit.
    @Override
    public void start() throws Exception {
        super.start() ;

        MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem();
        if (!has_explicit_target_)
            target_ = me.getPosition() ;
    }

    /// \brief Called each robot loop to update the motor power based on the PID controller.
    @Override
    public void run() {
        MotorEncoderSubsystem me = (MotorEncoderSubsystem)getSubsystem();
        double out = pid_.getOutput(target_, me.getPosition(), me.getRobot().getDeltaTime()) ;
        me.setPower(out) ;
    }

    /// \brief Cancel this action and sets the motor power to zero.
    @Override
    public void cancel() {
        super.cancel() ;
        getSubsystem().setPower(0.0) ;
    }

    /// \brief Returns a human readable string that describes the action
    /// \returns a human readable string that describes the action
    @Override
    public String toString(int indent) {
        String ret = prefix(indent) + "MotorEncoderHoldAction (" + getSubsystem().getName() + ")";
        if (has_explicit_target_)
            ret += ", explicit" ;
        else
            ret += ", implicit" ;
        ret += ", target=" + Double.toString(target_) ;
        return ret ;
    }

} ;