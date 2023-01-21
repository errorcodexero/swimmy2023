package org.xero1425.base.subsystems.motorsubsystem ;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

/// \file

/// \brief This class is a power action for the MotorEncoderSubsystem that applies a fixed power to the
/// motor and provides plots of the output in real world units
public class MotorEncoderPowerAction extends MotorPowerAction
{
    // The start time for a timed action
    private double start_ ;

    // The plot ID for the action
    private int plot_id_ ;

    // The columns to plot
    private static String[] plot_columns_ = { "time (s)","pos (m)","vel (m/s)","accel (m/s/s)","out (v)","encoder (ticks)" } ;

    /// \brief Create the MotorEncoderPowerAction that applies a fixed power value then is done
    /// \param motor the subsystem to apply the action to
    /// \param power the power to apply to the motor
    public MotorEncoderPowerAction(MotorEncoderSubsystem motor, double power) {
        super(motor, power);
        plot_id_ = -1 ;
    }

    /// \brief Create the MotorEncoderPowerAction that applies a fixed power value then is done
    /// \param motor the subsystem to apply the action to
    /// \param power a string that names the settings value in the settings file that contains the power value
    public MotorEncoderPowerAction(MotorEncoderSubsystem motor, String power)
            throws BadParameterTypeException, MissingParameterException {
        super(motor, power);
        plot_id_ = -1 ;
    }

    /// \brief Create the MotorEncoderPowerAction that applies the power for a fixed 
    /// amount of time then sets the power to zero.
    /// \param motor the subsystem to apply the action to
    /// \param power the power to apply to the motor
    /// \param duration the amount of time to apply the power  
    public MotorEncoderPowerAction(MotorEncoderSubsystem motor, double power, double duration) {
        super(motor, power, duration);
        plot_id_ = motor.initPlot(toString()) ;
    }

    /// \brief Create the MotorEncoderPowerAction that applies the power for a fixed 
    /// amount of time then sets the power to zero.
    /// \param motor the subsystem to apply the action to
    /// \param power a string that names the settings value in the settings file that contains the power value
    /// \param duration a string that names the settings value in the settings file that contains the duration value    
    public MotorEncoderPowerAction(MotorEncoderSubsystem motor, String power, String duration)
            throws BadParameterTypeException, MissingParameterException {

        super(motor, power, duration);
        plot_id_ = motor.initPlot(toString()) ;        
    }

    /// \brief Start the action by applying the power requested
    @Override
    public void start() throws Exception {
        super.start() ;

        start_ = getSubsystem().getRobot().getTime() ;
        getSubsystem().startPlot(plot_id_, plot_columns_);
    }

    /// \brief Called each robot loop.  Calls the base class to perform the action and then
    /// sends the plot data to the plot manager.
    @Override
    public void run() {
        super.run() ;

        Double [] data = new Double[plot_columns_.length] ;
        data[0] = getSubsystem().getRobot().getTime() - start_ ;
        data[1] = ((MotorEncoderSubsystem)(getSubsystem())).getPosition() ;
        data[2] = ((MotorEncoderSubsystem)(getSubsystem())).getVelocity() ;
        data[3] = ((MotorEncoderSubsystem)(getSubsystem())).getAcceleration() ;
        data[4] = getSubsystem().getPower() ;
        data[5] = ((MotorEncoderSubsystem)(getSubsystem())).getEncoderRawCount() ;
        getSubsystem().addPlotData(plot_id_, data);
        
        if (isDone())
            getSubsystem().endPlot(plot_id_) ;
    }

    /// \brief Cancel the action, settings the power to zero
    @Override
    public void cancel() {
        super.cancel() ;
        getSubsystem().endPlot(plot_id_) ;        
    }

    /// \brief Returnsa human readable string describing the action
    /// \returns a human readable string describing the action
    @Override
    public String toString() {
        String ret ;

        ret = "MotorEncoderPowerAction " + getSubsystem().getName() ;
        if (isTimed())
            ret += " " + getPower() + " " + getDuration() ;

        return ret ;
    }

}