package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.plotting.PlotDataSource;
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
    private Trajectory.State current_state_ ;
    private Pose2d start_position_ ;
    private Pose2d target_position_ ;
    private double trajectory_start_time_ ;
    private XeroTimer timer_ ;

    private Rotation2d facing_ ;
    private double maxa_ ;
    private double maxv_ ;

    private int plot_id_ ;
    private PlotDataSource plot_src_ ;


    
    public SwerveDriveToPoseAction(SwerveBaseSubsystem subsys, Pose2d pose2d, double maxa, double maxv) throws BadParameterTypeException, MissingParameterException {
        this(subsys) ;

        start_position_ = subsys.getPose();
        target_position_ = pose2d ;

        maxa_ = maxa ;
        maxv_ = maxv ;


    }

    public SwerveDriveToPoseAction(SwerveBaseSubsystem subsys, Pose2d pose2d) throws BadParameterTypeException, MissingParameterException {
        this(subsys) ;

        start_position_ = subsys.getPose();
        target_position_ = pose2d ;

        maxa_ = Double.MAX_VALUE;
        maxv_ = Double.MAX_VALUE;


    }

    public SwerveDriveToPoseAction(SwerveBaseSubsystem subsys, Pose2d[] endpoints, Rotation2d facing) throws BadParameterTypeException, MissingParameterException {
        this(subsys) ;

        start_position_ = endpoints[0];
        target_position_ = endpoints[1];
        facing_ = facing ;


        maxa_ = Double.MAX_VALUE;
        maxv_ = Double.MAX_VALUE;


    }

    public SwerveDriveToPoseAction(SwerveBaseSubsystem subsys, Pose2d[] endpoints, Rotation2d facing, double maxa, double maxv) throws BadParameterTypeException, MissingParameterException {
        this(subsys) ;

        start_position_ = endpoints[0];
        target_position_ = endpoints[1];
        facing_ = facing ;


        maxa_ = maxa ;
        maxv_ = maxv ;


    }    
    private SwerveDriveToPoseAction(SwerveBaseSubsystem subsys) throws BadParameterTypeException, MissingParameterException {
        super(subsys);


        createPlotDataSource();
        timer_ = new XeroTimer(subsys.getRobot(), "drivetimer", 1.0);
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

        plot_id_ = getSubsystem().initPlot("DriveToPose", plot_src_) ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        getSubsystem().startPlot(plot_id_);
        trajectory_ = getSubsystem().createTrajectory(start_position_, target_position_, maxa_, maxv_) ;
        trajectory_start_time_ = getSubsystem().getRobot().getTime() ;
    }

    @Override
    public void run() throws Exception {
        super.run();

        double deltat = getSubsystem().getRobot().getTime() - trajectory_start_time_ ;

        current_state_ = trajectory_.sample(getSubsystem().getRobot().getTime() - trajectory_start_time_) ;
        Rotation2d face = (facing_ == null) ? target_position_.getRotation() : facing_ ;
        ChassisSpeeds speed = controller().calculate(getSubsystem().getPose(), current_state_, face) ;
        getSubsystem().drive(speed) ;

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID());
        logger.add("DriveToPose:");
        logger.add("target", current_state_.poseMeters);
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
