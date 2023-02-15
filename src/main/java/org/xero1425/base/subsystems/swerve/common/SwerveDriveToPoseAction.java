package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

public class SwerveDriveToPoseAction extends SwerveHolonomicControllerAction {
    private Trajectory trajectory_ ;
    private Pose2d target_position_ ;
    private double trajectory_start_time_ ;

    private int plot_id_ ;
    private Double[] plot_data_ ;

    private static final String [] columns_ = {
        "time",
        "tx (m)", "ty (m)", "ta (deg)",
        "ax (m)", "ay (m)", "aa (deg)"
    } ;
    
    public SwerveDriveToPoseAction(SwerveBaseSubsystem subsys, Pose2d pose2d) throws BadParameterTypeException, MissingParameterException {
        super(subsys) ;

        target_position_ = pose2d ;
        plot_data_ = new Double[columns_.length] ;
        plot_id_ = getSubsystem().initPlot("DriveToPose") ;

        getSubsystem().startPlot(plot_id_, columns_);
    }

    @Override
    public void start() throws Exception {
        super.start();
        trajectory_ = getSubsystem().createTrajectory(target_position_) ;
        trajectory_start_time_ = getSubsystem().getRobot().getTime() ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        double deltat = getSubsystem().getRobot().getTime() - trajectory_start_time_ ;

        Trajectory.State st = trajectory_.sample(getSubsystem().getRobot().getTime() - trajectory_start_time_) ;
        ChassisSpeeds speed = controller().calculate(getSubsystem().getPose(), st, target_position_.getRotation()) ;
        getSubsystem().drive(speed) ;

        Pose2d actual = getSubsystem().getPose() ;
        int i = 0 ;
        plot_data_[i++] = getSubsystem().getRobot().getTime() - trajectory_start_time_;
        plot_data_[i++] = st.poseMeters.getX() ;
        plot_data_[i++] = st.poseMeters.getY() ;
        plot_data_[i++] = st.poseMeters.getRotation().getDegrees() ;
        plot_data_[i++] = actual.getX() ;
        plot_data_[i++] = actual.getY() ;
        plot_data_[i++] = actual.getRotation().getDegrees() ;
        getSubsystem().addPlotData(plot_id_, plot_data_) ;

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID());
        logger.add("DriveToPose:");
        logger.add("target", st.poseMeters);
        logger.add("actual", getSubsystem().getPose());
        logger.endMessage();

        if (deltat >= trajectory_.getTotalTimeSeconds() && controller().atReference()) {
            getSubsystem().endPlot(plot_id_);
            setDone() ;
            getSubsystem().drive(new ChassisSpeeds());
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveDriveToPoseAction " + target_position_.getTranslation().toString() + ", " + 
                target_position_.getRotation().getDegrees() + " deg" ;
    }
}
