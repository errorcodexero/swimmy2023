package org.xero1425.base.subsystems.tankdrive;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

/// \file

/// \brief This class just provides a constant power to the tank drive motors.
/// It has two modes.  In the first mode, the power is assigned and the action completes
/// immediately.  In the second mode, the requested power is assigned to the tank drive
/// subsystem for a fixed amount of time and the action completes after the time expires.
public class TankDrivePowerAction extends TankDriveAction {
    // The power to assign to the left side of the drive base 
    private double left_ ;

    // The power to assign to the right side of the drive base
    private double right_ ;

    // The time when this action started if it is the timed version
    private double start_ ;

    // The desired duration if this is the timed version
    private double duration_ ;

    // If true, this is the timed version of this action.
    private boolean timed_ ;

    // The ID of the plot to generate
    private int plot_id_ ;

    // An index that is incremented each time this action is created to ensure each action has a unique plot name
    private static int plot_number_ = 0 ;

    // The set of columns to plot
    private static final String [] plot_columns_ = { "time (sec)", "dist (m)", "velocity (m/s)", "acceleration (m/s/s)", "lticks (ticks)", "rticks (ticks)", "left (v)", "right (v)" } ;

    /// \brief Create the action.  Once power is assigned to the drive base, this action is complete.
    /// \param drive the tankdrive subsystem
    /// \param left the power to apply to the left side of the drive base
    /// \param right the power to apply to the right side of the drive base
    public TankDrivePowerAction(TankDriveSubsystem drive, double left, double right) {
        super(drive);

        left_ = left ;
        right_ = right ;
        timed_ = false;
    }

    /// \brief Create the action.  Once power is assigned to the drive base, this action is complete.
    /// \param drive the tankdrive subsystem    
    /// \param left the name of a setting in the settings file that contains the power for the left side of the drive base
    /// \param right the name of a setting in the settings file that contains the power for the right side of the drive base
    public TankDrivePowerAction(TankDriveSubsystem drive, String left, String right)
            throws BadParameterTypeException, MissingParameterException {
        super(drive);

        left_ = drive.getSettingsValue(left).getDouble();
        right_ = drive.getSettingsValue(right).getDouble() ;
        timed_ = false;
    }

    /// \brief Create the action. The action runs until the duration has elapsed and then the power is set to zero.
    /// \param drive the tankdrive subsystem    
    /// \param left the power to apply to the left side of the drive base
    /// \param right the power to apply to the right side of the drive base    
    /// \param duration the amount of time to apply this power to the drive base
    public TankDrivePowerAction(TankDriveSubsystem drive, double left, double right, double duration) {
        super(drive);

        left_ = left ;
        right_ = right ;
        duration_ = duration;
        timed_ = true;
        plot_id_ = drive.initPlot("tankdrivepower_" + Integer.toString(plot_number_++));
    }

    /// \brief Create the action. The action runs until the duration has elapsed and then the power is set to zero.
    /// \param drive the tankdrive subsystem    
    /// \param left the name of a setting in the settings file that contains the power for the left side of the drive base
    /// \param right the name of a setting in the settings file that contains the power for the right side of the drive base
    /// \param duration the name of a setting in the settings file that contains the duration for the action
    public TankDrivePowerAction(TankDriveSubsystem drive, String left, String right, String duration)
            throws BadParameterTypeException, MissingParameterException {

        super(drive);
        left_ = drive.getSettingsValue(left).getDouble();
        right_ = drive.getSettingsValue(right).getDouble();
        duration_ = drive.getSettingsValue(duration).getDouble();
        timed_ = true;
        plot_id_ = drive.initPlot("tankdrivepower_"+ Integer.toString(plot_number_++));        
    }

    /// \brief Start the action by applying the desired power to the left and right sides of the
    /// drivebase.  If this action is not timed, then the action is completed in this method.  Otherwise
    /// the action is completed when the duration expires.
    @Override
    public void start() throws Exception {
        super.start() ;

        try {
            getSubsystem().setPower(left_, right_) ;
            if (timed_)
                start_ = getSubsystem().getRobot().getTime() ;
            else
                setDone() ;
        }
        catch(Exception ex) {
        }

        if (timed_)
            getSubsystem().startPlot(plot_id_, plot_columns_) ;
    }

    /// \brief Called each robot loop to manage the action.  This is called for timed actions only
    /// and marks the action as done when the duration expires.
    @Override
    public void run() {
        if (timed_) {
            if (getSubsystem().getRobot().getTime() - start_ > duration_)
            {
                try 
                {
                    getSubsystem().setPower(0.0, 0.0) ;
                }
                catch(Exception ex)
                {
                }
                setDone() ;
                getSubsystem().endPlot(plot_id_) ;
            }

            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            logger.add("dbpower").add("lticks", getSubsystem().getLeftTick()).add("rticks", getSubsystem().getRightTick()) ;
            logger.endMessage();

            Double[] data = new Double[plot_columns_.length] ;
            data[0] = getSubsystem().getRobot().getTime() - start_ ;
            data[1] = getSubsystem().getDistance() ;
            data[2] = getSubsystem().getVelocity() ;
            data[3] = getSubsystem().getAcceleration() ;
            data[4] = (double)getSubsystem().getLeftTick() ;
            data[5] = (double)getSubsystem().getRightTick() ;
            data[6] = left_ ;
            data[7] = right_ ;
            getSubsystem().addPlotData(plot_id_, data);
        }
    }

    /// \brief Cancel the action, setting the power to zero
    @Override
    public void cancel() {
        super.cancel() ;

        try {
            getSubsystem().setPower(0.0, 0.0) ;
        }
        catch(Exception ex)
        {
        }        
    }

    /// \brief Returns a human readable string describing the action
    /// \returns a human readable string describing the action
    public String toString(int indent) {
        String ret = prefix(indent) + "TankDrivePowerAction" ;
        ret += " left=" + Double.toString(left_) ;
        ret += " right=" + Double.toString(right_) ;
        if (timed_)
            ret += " duration=" + Double.toString(duration_) ;

        return ret ;
    }
} ;
