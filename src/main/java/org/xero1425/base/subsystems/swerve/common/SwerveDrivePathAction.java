package org.xero1425.base.subsystems.swerve.common;

import java.util.List;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.misc.BadParameterTypeException;
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
    private Pose2d last_vision_pose_ ;
    private XeroTimer timer_ ;
    private Pose2d start_ ;
    private Pose2d end_ ;

    private int plot_id_ ;
    private Double[] plot_data_ ;

    private static final String [] columns_ = {
        "time",
        "tx (m)", "ty (m)", "ta (deg)",
        "ax (m)", "ay (m)", "aa (deg)",
        "vx (m)", "vy (m)", "va (deg)"
    } ;

    public SwerveDrivePathAction(SwerveBaseSubsystem sub, Pose2d start, List<Translation2d> interior, Pose2d end, Rotation2d facing) throws BadParameterTypeException, MissingParameterException {
        super(sub) ;

        TrajectoryConfig config = new TrajectoryConfig(getSubsystem().getMaxVelocity(), getSubsystem().getMaxAccel()) ;
        config.setKinematics(getSubsystem().getKinematics());
        trajectory_ = TrajectoryGenerator.generateTrajectory(start, interior, end, config) ;

        facing_ = facing;
        timer_ = new XeroTimer(sub.getRobot(), "drivetimer", 1.0);

        plot_data_ = new Double[columns_.length] ;
        plot_id_ = getSubsystem().initPlot("SwerveDrivePathAction") ;

        start_ = start ;
        end_ = end ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
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
        plot_data_[i++] = st.poseMeters.getRotation().getDegrees() ;
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

        if (deltat >= trajectory_.getTotalTimeSeconds() && controller().atReference()) {
            getSubsystem().endPlot(plot_id_);
            setDone() ;
            getSubsystem().drive(new ChassisSpeeds());
        }
        else if (deltat >= trajectory_.getTotalTimeSeconds() && timer_.isExpired()) {
            getSubsystem().endPlot(plot_id_);
            setDone() ;
            getSubsystem().drive(new ChassisSpeeds());
        }
    }

    public String toString(int indent) {
        return spaces(indent) + "SwerveDrivePathAction: from " + 
                start_.getTranslation().toString() + " to " + end_.getTranslation().toString() ;
    }
}
