package org.xero1425.base.subsystems.motorsubsystem ;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

/// \file

/// \brief This action applies a given power to a motor.  The action can apply the power and end immediately, or
/// it can apply the power for a given duration of time and then stop the motor when done.
public class MotorPowerAction extends MotorAction {
    // The power to apply
    private double power_ ;

    // For the timed version, when the action started
    private double start_ ;

    // For the timed version, the duration of the action
    private double duration_ ;

    // If true, this action is timed
    private boolean timed_ ;

    /// \brief Create a new MotorPowerAction that just applies a given power to the motor and is done.  This
    /// action leaves the motor running with the given power.
    /// \param motor the motor subsystem for the action
    /// \param power the power to apply to the motor
    public MotorPowerAction(MotorSubsystem motor, double power) {
        super(motor);

        power_ = power;
        timed_ = false;
    }

    /// \brief Create a new MotorPowerAction that just applies a given power to the motor and is done.  This
    /// action leaves the motor running with the given power.
    /// \param motor the motor subsystem for the action
    /// \param power the string giving the name of the entry in the settings file for the power
    public MotorPowerAction(MotorSubsystem motor, String power)
            throws BadParameterTypeException, MissingParameterException {
        super(motor);

        power_ = motor.getSettingsValue(power).getDouble();
        timed_ = false;
    }

    /// \brief Create a new MotorPowerAction that applies a power to the motor for a specific duration in time and then
    /// turn the motor off.
    /// \param motor the motor subsystem for the action
    /// \param power the power to apply to the motor    
    /// \param duration the amount of time to apply the power
    public MotorPowerAction(MotorSubsystem motor, double power, double duration) {
        super(motor);

        power_ = power;
        duration_ = duration;
        timed_ = true;
    }

    /// \brief Create a new MotorPowerAction that applies a power to the motor for a specific duration in time and then
    /// turn the motor off.
    /// \param motor the motor subsystem for the action
    /// \param power the string giving the name of the entry in the settings file for the power
    /// \param duration the string giving the name of the entry in the settings file for the duration
    public MotorPowerAction(MotorSubsystem motor, String power, String duration)
            throws BadParameterTypeException, MissingParameterException {

        super(motor);
        power_ = motor.getSettingsValue(power).getDouble();
        duration_ = motor.getSettingsValue(duration).getDouble();
        timed_ = true;
    }

    /// \brief Return the power applied to the motor
    /// \returns the power applied to the motor
    public double getPower() {
        return power_ ;
    }

    /// \brief Return the duration of the action
    /// \returns the duration of the action
    public double getDuration() {
        return duration_ ;
    }

    /// \brief returns true if the action is a timed action
    /// \returns true if the action is a timed action
    public boolean isTimed() {
        return timed_ ;
    }

    /// \brief Start the action by applying the power to the motor
    @Override
    public void start() throws Exception {
        super.start() ;

        try {
            getSubsystem().setPower(power_) ;
            if (timed_)
                start_ = getSubsystem().getRobot().getTime() ;
            else
                setDone() ;
        }
        catch(Exception ex) {
        }
    }

    /// \brief Run the action, my monitoring the time and ending the action when
    /// the time the power applied meets the duration.
    @Override
    public void run() {
        double now = getSubsystem().getRobot().getTime() ;
        if (timed_) {
            if (now - start_ > duration_)
            {
                getSubsystem().setPower(0.0) ;
                setDone() ;
            }
        }
    }

    /// \brief Cancel the action, setting the motor power to zero
    @Override
    public void cancel() {
        super.cancel() ;
        getSubsystem().setPower(0.0) ;
    }

    /// \brief Returns a human readable string representing the action
    /// \returns a human readable string representing the action
    public String toString(int indent) {
        String ret = prefix(indent) + "MotorPowerAction, " + getSubsystem().getName();
        ret += " , power=" + Double.toString(power_) ;
        if (timed_)
            ret += " duration=" + Double.toString(duration_) ;

        return ret ;
    }
} ;