package org.xero1425.base.subsystems.swerve.common;

import java.util.List;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.plotting.PlotDataSource;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class SwerveDrivePathAction extends SwerveHolonomicControllerAction {
    private double trajectory_start_time_ ;
    private Trajectory trajectory_ ;
    private Rotation2d facing_ ;
    private XeroTimer timer_ ;
    private Pose2d start_ ;
    private Pose2d end_ ;
    private double maxa_ ;
    private double maxv_ ;
    private Trajectory.State current_state_ ;

    private int plot_id_ ;
    private PlotDataSource plot_src_ ;

    public SwerveDrivePathAction(SwerveBaseSubsystem sub, Pose2d start, List<Translation2d> interior, Pose2d end, Rotation2d facing, double maxa, double maxv) throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        if (maxa == Double.MAX_VALUE) {
            maxa = getSubsystem().getMaxAccel() ;
        }

        if (maxv == Double.MAX_VALUE) {
            maxv = getSubsystem().getMaxVelocity();
        }

        TrajectoryConfig config = new TrajectoryConfig(maxv, maxa) ;
        config.setKinematics(getSubsystem().getKinematics());
        trajectory_ = TrajectoryGenerator.generateTrajectory(start, interior, end, config) ;

        facing_ = facing;
        timer_ = new XeroTimer(sub.getRobot(), "drivetimer", 1.0);

        createPlotDataSource();

        start_ = start ;
        end_ = end ;

        maxa_ = maxa ;
        maxv_ = maxv ;
    }

    private void createPlotDataSource() {
        plot_src_ = new PlotDataSource() ;

        plot_src_.addDataElement("time", () -> { return getSubsystem().getRobot().getTime() - trajectory_start_time_ ;});

        plot_src_.addDataElement("tx (m)", () -> { return current_state_.poseMeters.getX() ; });
        plot_src_.addDataElement("ty (m)", () -> { return current_state_.poseMeters.getY() ; });
        plot_src_.addDataElement("ta (deg)", () -> { return current_state_.poseMeters.getRotation().getDegrees() ; });

        plot_src_.addDataElement("ax (m)", () -> { return getSubsystem().getPose().getX() ; });
        plot_src_.addDataElement("ay (m)", () -> { return getSubsystem().getPose().getY() ; });
        plot_src_.addDataElement("aa (deg)", () -> { return getSubsystem().getPose().getRotation().getDegrees() ; });

        plot_id_ = getSubsystem().initPlot("SwerveDrivePathAction", plot_src_) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        trajectory_start_time_ = getSubsystem().getRobot().getTime() ;
        getSubsystem().startPlot(plot_id_);
    }

    @Override
    public void run()  throws Exception {
        double deltat = getSubsystem().getRobot().getTime() - trajectory_start_time_ ;
        
        current_state_ = trajectory_.sample(getSubsystem().getRobot().getTime() - trajectory_start_time_) ;
        ChassisSpeeds speed = controller().calculate(getSubsystem().getPose(), current_state_, facing_) ;
        getSubsystem().drive(speed) ;

        if (deltat >= trajectory_.getTotalTimeSeconds() && !timer_.isRunning()) {
            timer_.start();
        }

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger();
        if (deltat >= trajectory_.getTotalTimeSeconds() && controller().atReference()) {
            getSubsystem().endPlot(plot_id_);
            setDone() ;
            getSubsystem().drive(new ChassisSpeeds());

            logger.startMessage(MessageType.Info);
            logger.add("SwerveDrivePathAction: complete due robot at endpoint");
            logger.add("target pose", end_);
            logger.add("actual pose", getSubsystem().getPose());
            logger.endMessage();
        }
        else if (deltat >= trajectory_.getTotalTimeSeconds() && timer_.isExpired()) {
            getSubsystem().endPlot(plot_id_);
            setDone() ;
            getSubsystem().drive(new ChassisSpeeds());

            logger.startMessage(MessageType.Info);
            logger.add("SwerveDrivePathAction: complete due to timer expiration");
            logger.endMessage();
        }
    }

    public String toString(int indent) {
        return spaces(indent) + "SwerveDrivePathAction(maxv " + maxv_ + ", maxa " + maxa_ + ")" +
                ": from " + start_.getTranslation().toString() + " to " + end_.getTranslation().toString() ;
    }
}
