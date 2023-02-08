package org.xero1425.base.subsystems.swerve.common;

import java.util.ArrayList;
import java.util.List;

import org.xero1425.base.IVisionLocalization;
import org.xero1425.base.IVisionLocalization.LocationData;
import org.xero1425.base.IVisionLocalization.LocationType;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MinMaxData;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public abstract class SwerveBaseSubsystem extends DriveBaseSubsystem {
    private int plotid_ ;
    private double plotstart_ ;
    private Double[] plotdata_ ;
    private static final String [] columns_ = {
        "time",
        "fl-ang-t (deg)", "fl-ang-a (deg)","fl-drv-t (m/s)","fl-drv-a (m/s)",
        "fr-ang-t (deg)", "fr-ang-a (deg)","fr-drv-t (m/s)","fr-drv-a (m/s)",
        "bl-ang-t (deg)", "bl-ang-a (deg)","bl-drv-t (m/s)","bl-drv-a (m/s)",
        "br-ang-t (deg)", "br-ang-a (deg)","br-drv-t (m/s)","br-drv-a (m/s)",
    } ;
    
    private int index_ ;

    private IVisionLocalization vision_ ;
    private SwerveDriveKinematics kinematics_ ;
    private SwerveDrivePoseEstimator estimator_ ;

    private double [] angles_ ;
    private double [] powers_ ;

    private double width_ ;
    private double length_ ;
    private double maxv_ ;
    private double maxa_ ;
    private double rotate_angle_ ;
    private Pose2d last_pose_ ;

    private MinMaxData velocity_ ;
    private MinMaxData rotational_velocity_ ;
   
    static public final int FL = 0;                                                             // Index of the front left module
    static public final int FR = 1;                                                             // Index of the front right module
    static public final int BL = 2;                                                             // Index of the back left module
    static public final int BR = 3;                                                             // Index of the back right module

    public SwerveBaseSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name) ;

        angles_ = new double[4] ;
        powers_ = new double[4] ;

        // Note: Change to 1 to get previous behavior
        velocity_ = new MinMaxData(10) ;
        rotational_velocity_ = new MinMaxData(10) ;

        for(int i = 0 ; i < 4 ; i++) {
            angles_[i] = 0.0 ;
            powers_[i] = 0.0 ;
        }

        plotdata_ = new Double[columns_.length] ;
        plotid_ = -1 ;
       
        width_ = getSettingsValue("physical:width").getDouble() ;
        length_ = getSettingsValue("physical:length").getDouble() ;
        maxv_ = getSettingsValue("physical:path:maxv").getDouble() ;
        maxa_ = getSettingsValue("physical:path:maxa").getDouble() ;

        //
        // Calculate the angle for the velocity vector to rotate the robot
        //
        rotate_angle_ = Math.toDegrees(Math.atan2(length_, width_)) ;

        kinematics_ = new SwerveDriveKinematics(new Translation2d(getWidth() / 2.0, getLength() / 2.0), new Translation2d(getWidth() / 2.0, -getLength() / 2.0), 
                        new Translation2d(-getWidth() / 2.0, getLength() / 2.0), new Translation2d(-getWidth() / 2.0, -getLength() / 2.0)) ;

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        shuffleboardTab.addNumber("Heading", () -> getHeading().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> getPose().getX());
        shuffleboardTab.addNumber("Pose Y", () -> getPose().getY());

        last_pose_ = new Pose2d() ;
    }

    public void setVision(IVisionLocalization vision) {
        vision_ = vision ;
    }

    public String getStatus() {
        String st = "Current Pose " + getPose().toString() ;
        return st ;
    }

    protected void createOdometry() throws Exception {
        SwerveModulePosition [] poss = new SwerveModulePosition[4] ;
        poss[0] = getModulePosition(FL) ;
        poss[1] = getModulePosition(FR) ;
        poss[2] = getModulePosition(BL) ;
        poss[3] = getModulePosition(BR) ;

        Rotation2d heading = Rotation2d.fromDegrees(gyro().getYaw()) ;
        
        try {
            estimator_ = new SwerveDrivePoseEstimator(kinematics_, heading, poss, last_pose_) ;
        } catch(Exception ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("exception thrown creating SwerveDrivePoseEstimator - " + ex.getMessage()) ;
            logger.endMessage();
            throw ex ;
        }
    }

    // Control the swerve drive by settings a ChassisSppeds object
    public abstract void drive(ChassisSpeeds speeds) ;

    // Control the swerve drive by supplying raw targets.  If power is true then the values
    // supplied by the speeds_power array are power numbers to go directly to the drive motors.
    // If power is false, these are speed numbers to feed into the drive motor PID controller 
    // for the modules.
    public abstract void setRawTargets(boolean power, double [] angles, double [] speeds_powers) ;

    public abstract SwerveModuleState getModuleState(int which) ;

    public abstract SwerveModulePosition getModulePosition(int which) ;

    public abstract SwerveModuleState getModuleTarget(int which) ;

    public void stop() throws BadMotorRequestException, MotorRequestFailedException {
        setRawTargets(false, powers_, angles_);
    }

    public void addVisionMeasurement(Pose2d vision, double when) {
        estimator_.addVisionMeasurement(vision, when);
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        SwerveModulePosition [] poss = new SwerveModulePosition[4] ;
        poss[0] = getModulePosition(FL) ;
        poss[1] = getModulePosition(FR) ;
        poss[2] = getModulePosition(BL) ;
        poss[3] = getModulePosition(BR) ;
        estimator_.update(Rotation2d.fromDegrees(gyro().getYaw()), poss) ;

        boolean added = false;
        if (vision_ != null) {
            LocationData lc = vision_.getLocation() ;
            if (lc != null) {
                if (lc.type == LocationType.RobotFieldLocation) {
                    Pose2d p2d = lc.location.toPose2d() ;
                    addVisionMeasurement(p2d, lc.when) ;
                    added = true ;
                }
            }
        }
        putDashboard("addvision", DisplayType.Always, added);

        Pose2d p = getPose() ;
        double dist = p.getTranslation().getDistance(last_pose_.getTranslation()) ;
        double v = dist / getRobot().getDeltaTime() ;
        velocity_.addData(v);

        v = (p.getRotation().getDegrees() - last_pose_.getRotation().getDegrees()) / getRobot().getDeltaTime() ;
        rotational_velocity_.addData(v) ;

        last_pose_ = p ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    protected SwerveDriveKinematics getKinematics() {
        return kinematics_ ;
    }

    public Pose2d getPose() {
        return estimator_.getEstimatedPosition() ;
    }

    public void setPose(Pose2d pose) {
        Rotation2d rot = Rotation2d.fromDegrees(gyro().getYaw()) ;
        
        SwerveModulePosition [] poss = new SwerveModulePosition[4] ;
        poss[0] = getModulePosition(FL) ;
        poss[1] = getModulePosition(FR) ;
        poss[2] = getModulePosition(BL) ;
        poss[3] = getModulePosition(BR) ;
        estimator_.resetPosition(rot, poss, pose) ;
        gyro().reset() ;
    }

    // This is a hack for this one event.  Need to rethink this after block party
    public double getVelocity() {
        return velocity_.getMax() ;
    }

    // This is a hack for this one event.  Need to rethink this after block party
    public double getRotationalVelocity() {
        return rotational_velocity_.getMax() ;
    }

    public Rotation2d getHeading() {
        return getPose().getRotation() ;
    }

    public int getModuleCount() {
        return 4 ;
    }
    
    /// \brief return the base angle for a vector to rotate the robot.
    /// \returns the base angle for a vector to rotate the robot
    public double getPHI() {
        return rotate_angle_ ;
    }

    public double getWidth() {
        return width_ ;
    }

    public double getLength() {
        return length_ ;
    }
   
    public void startSwervePlot(String name) {
        if (plotid_ == -1) {
            plotid_ = initPlot(name) ;
        }

        startPlot(plotid_, columns_);
        plotstart_ = getRobot().getTime();
    }

    public void endSwervePlot() {
        endPlot(plotid_);
        plotid_ = -1 ;
    }

    public Trajectory createTrajectory(List<Pose2d> waypoints) {
        TrajectoryConfig config = new TrajectoryConfig(maxv_, maxa_) ;
        config.setKinematics(kinematics_) ;
        return TrajectoryGenerator.generateTrajectory(waypoints, config) ;
    }
   
    public Trajectory createTrajectory(Pose2d end) {
        List<Pose2d> waypoints = new ArrayList<Pose2d>() ;
        waypoints.add(getPose()) ;
        waypoints.add(end) ;
        return createTrajectory(waypoints) ;
    }

    protected void newPlotData() {
        index_ = 0 ;
        
        putData(getRobot().getTime() - plotstart_) ;

        putData(getModuleTarget(FL).angle.getDegrees()) ;
        putData(getModuleState(FL).angle.getDegrees()) ;
        putData(getModuleTarget(FL).speedMetersPerSecond) ;
        putData(getModuleState(FL).speedMetersPerSecond) ;

        putData(getModuleTarget(FR).angle.getDegrees()) ;
        putData(getModuleState(FR).angle.getDegrees()) ;
        putData(getModuleTarget(FR).speedMetersPerSecond) ;
        putData(getModuleState(FR).speedMetersPerSecond) ;

        putData(getModuleTarget(BL).angle.getDegrees()) ;
        putData(getModuleState(BL).angle.getDegrees()) ;
        putData(getModuleTarget(BL).speedMetersPerSecond) ;
        putData(getModuleState(BL).speedMetersPerSecond) ;
        
        putData(getModuleTarget(BR).angle.getDegrees()) ;
        putData(getModuleState(BR).angle.getDegrees()) ;
        putData(getModuleTarget(BR).speedMetersPerSecond) ;
        putData(getModuleState(BR).speedMetersPerSecond) ;

        addPlotData(plotid_, plotdata_);
    }

    private void putData(double d) {
        plotdata_[index_++] = d ;
    }
}
