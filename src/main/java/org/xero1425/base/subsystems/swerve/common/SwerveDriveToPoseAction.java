package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

public class SwerveDriveToPoseAction extends SwerveHolonomicControllerAction {
    private Trajectory trajectory_ ;
    private Pose2d start_position_ ;
    private Pose2d target_position_ ;
    private double trajectory_start_time_ ;
    private XeroTimer timer_ ;
    private Pose2d last_vision_pose_ ;
    private Rotation2d facing_ ;
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
    
    public SwerveDriveToPoseAction(SwerveBaseSubsystem subsys, Pose2d pose2d, double maxa, double maxv) throws BadParameterTypeException, MissingParameterException {
        super(subsys) ;

        start_position_ = subsys.getPose();
        target_position_ = pose2d ;
        plot_data_ = new Double[columns_.length] ;
        plot_id_ = getSubsystem().initPlot("DriveToPose") ;
        maxa_ = maxa ;
        maxv_ = maxv ;

        timer_ = new XeroTimer(subsys.getRobot(), "drivetimer", 1.0);
    }

    public SwerveDriveToPoseAction(SwerveBaseSubsystem subsys, Pose2d pose2d) throws BadParameterTypeException, MissingParameterException {
        super(subsys) ;

        start_position_ = subsys.getPose();
        target_position_ = pose2d ;
        plot_data_ = new Double[columns_.length] ;
        plot_id_ = getSubsystem().initPlot("DriveToPose") ;
        maxa_ = Double.MAX_VALUE;
        maxv_ = Double.MAX_VALUE;

        timer_ = new XeroTimer(subsys.getRobot(), "drivetimer", 1.0);
    }

    public SwerveDriveToPoseAction(SwerveBaseSubsystem subsys, Pose2d[] endpoints, Rotation2d facing) throws BadParameterTypeException, MissingParameterException {
        super(subsys) ;

        start_position_ = endpoints[0];
        target_position_ = endpoints[1];
        facing_ = facing ;
        plot_data_ = new Double[columns_.length] ;
        plot_id_ = getSubsystem().initPlot("DriveToPose") ;

        maxa_ = Double.MAX_VALUE;
        maxv_ = Double.MAX_VALUE;

        timer_ = new XeroTimer(subsys.getRobot(), "drivetimer", 1.0);
    }

    public SwerveDriveToPoseAction(SwerveBaseSubsystem subsys, Pose2d[] endpoints, Rotation2d facing, double maxa, double maxv) throws BadParameterTypeException, MissingParameterException {
        super(subsys) ;

        start_position_ = endpoints[0];
        target_position_ = endpoints[1];
        facing_ = facing ;
        plot_data_ = new Double[columns_.length] ;
        plot_id_ = getSubsystem().initPlot("DriveToPose") ;

        maxa_ = maxa ;
        maxv_ = maxv ;

        timer_ = new XeroTimer(subsys.getRobot(), "drivetimer", 1.0);
    }    

    @Override
    public void start() throws Exception {
        super.start();
        getSubsystem().startPlot(plot_id_, columns_);
        trajectory_ = getSubsystem().createTrajectory(start_position_, target_position_, maxa_, maxv_) ;
        trajectory_start_time_ = getSubsystem().getRobot().getTime() ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        double deltat = getSubsystem().getRobot().getTime() - trajectory_start_time_ ;

        Trajectory.State st = trajectory_.sample(getSubsystem().getRobot().getTime() - trajectory_start_time_) ;
        Rotation2d face = (facing_ == null) ? target_position_.getRotation() : facing_ ;
        ChassisSpeeds speed = controller().calculate(getSubsystem().getPose(), st, face) ;
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

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID());
        logger.add("DriveToPose:");
        logger.add("target", st.poseMeters);
        logger.add("actual", getSubsystem().getPose());
        logger.endMessage();

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

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveDriveToPoseAction " + target_position_.getTranslation().toString() + ", " + 
                target_position_.getRotation().getDegrees() + " deg" ;
    }
}
