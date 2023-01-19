package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.XeroPath;
import org.xero1425.misc.XeroPathSegment;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveHolonomicPathFollower extends SwerveDriveAction {

    private String pathname_ ;
    private HolonomicDriveController ctrl_ ;
    private XeroPath path_;
    private int index_ ;
    private boolean setpose_ ;

    private double start_ ;
    private int plot_id_ ;
    private Double[] plot_data_ ;

    private static final String [] columns_ = {
        "time",
        "tx (m)", "ty (m)", "ta (deg)",
        "ax (m)", "ay (m)", "aa (deg)"
    } ;

    public SwerveHolonomicPathFollower(SwerveBaseSubsystem sub, String pathname, boolean setpose) {
        super(sub) ;

        pathname_ = pathname ;
        setpose_ = setpose ;

        plot_data_ = new Double[columns_.length] ;
        plot_id_ = getSubsystem().initPlot("holonomic" + pathname_) ;
    }

    @Override
    public void start() throws Exception {
        double kp, ki, kd ;

        super.start() ;

        getSubsystem().startPlot(plot_id_, columns_);

        start_ = getSubsystem().getRobot().getTime() ;

        double maxv = getSubsystem().getSettingsValue("physical:max-angular-speed").getDouble() ;
        double maxa = getSubsystem().getSettingsValue("physical:max-angular-accel").getDouble() ;

        kp = getSubsystem().getSettingsValue("pid:xctrl:kp").getDouble() ;
        ki = getSubsystem().getSettingsValue("pid:xctrl:ki").getDouble() ;
        kd = getSubsystem().getSettingsValue("pid:xctrl:kd").getDouble() ;
        PIDController xctrl = new PIDController(kp, ki, kd) ;

        kp = getSubsystem().getSettingsValue("pid:yctrl:kp").getDouble() ;
        ki = getSubsystem().getSettingsValue("pid:yctrl:ki").getDouble() ;
        kd = getSubsystem().getSettingsValue("pid:yctrl:kd").getDouble() ;
        PIDController yctrl = new PIDController(kp, ki, kd) ;

        kp = getSubsystem().getSettingsValue("pid:rotation:kp").getDouble() ;
        ki = getSubsystem().getSettingsValue("pid:rotation:ki").getDouble() ;
        kd = getSubsystem().getSettingsValue("pid:rotation:kd").getDouble() ;
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxv, maxa) ;
        ProfiledPIDController thetactrl = new ProfiledPIDController(kp, ki, kd, constraints) ;
        thetactrl.enableContinuousInput(-Math.PI, Math.PI);
        
        ctrl_ = new HolonomicDriveController(xctrl, yctrl, thetactrl) ;
        ctrl_.setEnabled(true);

        ISettingsSupplier settings = getSubsystem().getRobot().getSettingsSupplier() ;
        double xytol = settings.get("subsystems:swervedrive:holonomic-path-following:xy-tolerance").getDouble() ;
        double angletol = settings.get("subsystems:swervedrive:holonomic-path-following:angle-tolerance").getDouble() ;
        ctrl_.setTolerance(new Pose2d(xytol, xytol, Rotation2d.fromDegrees(angletol))) ;

        path_ = getSubsystem().getRobot().getPathManager().getPath(pathname_);

        if (setpose_) {
            Pose2d pose = getPoseFromPath(0) ;
            getSubsystem().setPose(pose);

            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            logger.add("SwerveHolonomicPathFollower: Initial Pose ", pose.toString()) ;
            logger.endMessage();
        }

        index_ = 0 ;
    }

    @Override
    public void run() throws BadMotorRequestException, MotorRequestFailedException {

        if (index_ < path_.getTrajectoryEntryCount())
        {
            Pose2d target = getPoseFromPath(index_);

            MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
            logger.add("SwerveHolonomicPathFollower Target:").add("index", index_) ;
            logger.add(", target ") ;
            logger.add(target.getX()).add(" ").add(target.getY()) ;
            logger.add(" ").add(target.getRotation().getDegrees()) ;

            Pose2d actual = getSubsystem().getPose() ;
            logger.add(",actual ") ;
            logger.add(actual.getX()).add(" ").add(actual.getY()) ;
            logger.add(" ").add(actual.getRotation().getDegrees()) ;

            logger.endMessage();

            int i = 0 ;
            plot_data_[i++] = getSubsystem().getRobot().getTime() - start_ ;
            plot_data_[i++] = target.getX() ;
            plot_data_[i++] = target.getY() ;
            plot_data_[i++] = target.getRotation().getDegrees() ;
            plot_data_[i++] = actual.getX() ;
            plot_data_[i++] = actual.getY() ;
            plot_data_[i++] = actual.getRotation().getDegrees() ;
            getSubsystem().addPlotData(plot_id_, plot_data_) ;

            double velocity = getVelocityFromPath(index_) ;
            ChassisSpeeds speed = ctrl_.calculate(getSubsystem().getPose(), target, velocity, target.getRotation()) ;
            getSubsystem().drive(speed) ;
            index_++ ;
        }

        if (index_ >= path_.getTrajectoryEntryCount())
        {
            getSubsystem().endPlot(plot_id_);
            getSubsystem().drive(new ChassisSpeeds()) ;
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveHolonomicPathFollower " + pathname_ ;
    }

    private Pose2d getPoseFromPath(int index) {
        XeroPathSegment main = path_.getSegment(0, index) ;
        return new Pose2d(main.getX(), main.getY(), Rotation2d.fromDegrees(main.getRotation())) ;
    }

    private double getVelocityFromPath(int index) {
        XeroPathSegment main = path_.getSegment(0, index) ;
        return main.getVelocity() ;
    }
}
