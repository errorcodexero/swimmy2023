package org.xero1425.base.subsystems.swerve.common;

import java.util.List;

import org.xero1425.base.misc.XeroTimer;
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
    private TrajectoryConfig config_ ;
    private Rotation2d facing_ ;
    private Pose2d last_vision_pose_ ;
    private XeroTimer timer_ ;
    private Pose2d start_ ;
    private Pose2d end_ ;
    private List<Translation2d> interior_ ;
    private double svel_ ;
    private double evel_ ;
    private double maxa_ ;
    private double maxv_ ;

    private int plot_id_ ;
    private Double[] plot_data_ ;

    private static final String [] columns_ = {
        "time",
        "tx (m)", "ty (m)", "ta (deg)",
        "ax (m)", "ay (m)", "aa (deg)",
        "vx (m)", "vy (m)", "va (deg)"
    } ;

    public SwerveDrivePathAction(SwerveBaseSubsystem sub, Pose2d start, double svel, List<Translation2d> interior, Pose2d end, double evel, Rotation2d facing, double maxa, double maxv) throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        if (maxa == Double.MAX_VALUE) {
            maxa = getSubsystem().getMaxAccel() ;
        }

        if (maxv == Double.MAX_VALUE) {
            maxv = getSubsystem().getMaxVelocity();
        }

        facing_ = facing;
        timer_ = new XeroTimer(sub.getRobot(), "drivetimer", 0.2);

        plot_data_ = new Double[columns_.length] ;
        plot_id_ = getSubsystem().initPlot("SwerveDrivePathAction") ;

        start_ = start ;
        end_ = end ;
        svel_ = svel ;
        evel_ = evel ;
        interior_ = interior ;

        maxa_ = maxa ;
        maxv_ = maxv ;
    }

    public SwerveDrivePathAction(SwerveBaseSubsystem sub, List<Translation2d> interior, Pose2d end, double evel, Rotation2d facing, double maxa, double maxv) throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        if (maxa == Double.MAX_VALUE) {
            maxa = getSubsystem().getMaxAccel() ;
        }

        if (maxv == Double.MAX_VALUE) {
            maxv = getSubsystem().getMaxVelocity();
        }

        facing_ = facing;
        timer_ = new XeroTimer(sub.getRobot(), "drivetimer", 0.2);

        plot_data_ = new Double[columns_.length] ;
        plot_id_ = getSubsystem().initPlot("SwerveDrivePathAction") ;

        start_ = null ;
        end_ = end ;
        evel_ = evel ;
        interior_ = interior ;

        maxa_ = maxa ;
        maxv_ = maxv ;
    }    

    @Override
    public void start() throws Exception {
        super.start() ;

        if (start_ == null) {
            start_ = getSubsystem().getPose() ;
            svel_ = getSubsystem().getVelocity() ;
        }

        config_ = new TrajectoryConfig(maxv_, maxa_) ;
        config_.setStartVelocity(svel_) ;
        config_.setEndVelocity(evel_);
        config_.setKinematics(getSubsystem().getKinematics());
        trajectory_ = TrajectoryGenerator.generateTrajectory(start_, interior_, end_, config_) ;
        trajectory_start_time_ = getSubsystem().getRobot().getTime() ;
        getSubsystem().startPlot(plot_id_, columns_);
    }

    @Override
    public void run()  throws Exception {
        double deltat = getSubsystem().getRobot().getTime() - trajectory_start_time_ ;
        
        Trajectory.State st = trajectory_.sample(getSubsystem().getRobot().getTime() - trajectory_start_time_) ;
        ChassisSpeeds speed = controller().calculate(getSubsystem().getPose(), st, facing_) ;
        getSubsystem().drive(speed) ;

        Pose2d vpose = getSubsystem().getVisionPose() ;

        Pose2d actual = getSubsystem().getPose() ;
        int i = 0 ;
        plot_data_[i++] = getSubsystem().getRobot().getTime() - trajectory_start_time_;
        plot_data_[i++] = st.poseMeters.getX() ;
        plot_data_[i++] = st.poseMeters.getY() ;
        plot_data_[i++] = facing_.getDegrees();
        plot_data_[i++] = actual.getX() ;
        plot_data_[i++] = actual.getY() ;
        plot_data_[i++] = actual.getRotation().getDegrees() ;

        if (vpose != null) {
            plot_data_[i++] = vpose.getX() ;
            plot_data_[i++] = vpose.getY() ;
            plot_data_[i++] = vpose.getRotation().getDegrees() ;
            last_vision_pose_ = vpose ;
        }
        else if (last_vision_pose_ != null) {
            plot_data_[i++] = last_vision_pose_.getX() ;
            plot_data_[i++] = last_vision_pose_.getY() ;
            plot_data_[i++] = last_vision_pose_.getRotation().getDegrees() ;
        }
        else {
            plot_data_[i++] = 0.0 ;
            plot_data_[i++] = 0.0 ;
            plot_data_[i++] = 0.0 ;
        }

        getSubsystem().addPlotData(plot_id_, plot_data_) ;

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
        String ret ;

        if (start_ == null) {
            return spaces(indent) + "SwerveDrivePathAction(maxv " + maxv_ + ", maxa " + maxa_ + ")" +
                    ": from ROBOTPOS to " + end_.getTranslation().toString() ;
        }
        else {
            return spaces(indent) + "SwerveDrivePathAction(maxv " + maxv_ + ", maxa " + maxa_ + ")" +
                    ": from " + start_.getTranslation().toString() + " to " + end_.getTranslation().toString() ;
        }
    }
}
