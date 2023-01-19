package org.xero1425.base.subsystems.tankdrive;

/// \file

import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

/// \brief This action is used to characterize the scrub factor for a tankdrive base
/// This action rotates the robot in place through a given angle and measures the distance the wheels
/// traveled in a circle.  It compute the scrub factor which is the ratio of the actual distance traveled
/// versus the ideal distance traveled if the wheels had not friction. 
public class TankDriveScrubCharAction extends TankDriveAction {
    /// \brief Create the scrub action
    /// \param drive the tankdrive subsystem
    /// \param power the power to use in the action
    /// \param total the total angle to traverse
    public TankDriveScrubCharAction(final TankDriveSubsystem drive, final double power, final double total) {
        super(drive);
        power_ = power;
        total_ = total;

        plot_id_ = drive.initPlot("tankdrivescrub");
    }

    /// \brief start the action
    /// Applies power to the robot wheel in the opposite directions to cause the robot to rotate.  The robot rotates
    /// until the total angle given in the constructor has been covered.
    /// \throws Exception only if underlying hardware (like gyro) throws an exception
    @Override
    public void start() throws Exception {
        super.start();

        getSubsystem().setPower(-power_, power_);
        start_ = getSubsystem().getRobot().getTime();
        start_angle_ = getSubsystem().getTotalAngle();
        getSubsystem().startPlot(plot_id_, plot_columns_);
    }

    /// \brief Called once per robot loop to manage the scrub action
    @Override
    public void run() {

        final double angle = getSubsystem().getTotalAngle() - start_angle_;
        final double distl = getSubsystem().getLeftDistance();
        final double distr = getSubsystem().getRightDistance();

        if (Math.abs(angle) > total_) {
            final MessageLogger logger = getSubsystem().getRobot().getMessageLogger();
            setDone();
            getSubsystem().setPower(0.0, 0.0);

            final double avgc = (distl - distr) / 2.0;
            final double revs = angle / 360.0;
            final double effr = avgc / (Math.PI * revs);
            final double scrub = effr / getSubsystem().getWidth() ;

            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID());
            logger.add("Total Angle (NaVX) ").add(angle);
            logger.add(", left ").add(distl);
            logger.add(", right ").add(distr);
            logger.add(", effective Width ").add(effr * 2.0);
            logger.add(", scrub", scrub) ;
            logger.endMessage();
        } else {
            final Double[] data = new Double[7];
            data[0] = getSubsystem().getRobot().getTime() - start_;
            data[1] = getSubsystem().getAngle().getDegrees() ;
            data[4] = (double) getSubsystem().getLeftTick();
            data[5] = (double) getSubsystem().getRightTick();
            data[6] = power_;
            getSubsystem().addPlotData(plot_id_, data);
        }
    }

    /// \brief Cancel the current actop and stop the drivebase motors
    @Override
    public void cancel() {
        super.cancel();

        getSubsystem().setPower(0.0, 0.0);
        getSubsystem().endPlot(plot_id_);
    }

    /// \brief Returns a human readable string describing the action
    /// \returns a human readable string describing the action    
    public String toString(int indent) {
        String ret = prefix(indent) + "TankDriveScrubCharAction";
        ret += " power=" + Double.toString(power_);
        ret += " angle=" + Double.toString(total_);

        return ret;
    }

    private final double power_;
    private double start_;
    private double start_angle_;
    private final double total_;
    private final int plot_id_;
    private static String [] plot_columns_ = { "time", "angle", "lticks", "rticks", "power" } ;
} ;