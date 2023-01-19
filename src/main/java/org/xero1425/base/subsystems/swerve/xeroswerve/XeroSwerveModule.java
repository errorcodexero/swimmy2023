package org.xero1425.base.subsystems.swerve.xeroswerve;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.MotorController.EncoderUpdateFrequency;
import org.xero1425.base.motors.MotorController.PidType;
import org.xero1425.base.subsystems.motorsubsystem.EncoderConfigException;
import org.xero1425.base.subsystems.motorsubsystem.XeroEncoder;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;
import org.xero1425.misc.Speedometer;
import org.xero1425.misc.XeroMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class XeroSwerveModule {
    private XeroRobot robot_ ;
    private String name_ ;
    private MotorController steer_;
    private MotorController drive_;
    private XeroEncoder encoder_;
    private double steer_power_;
    private double drive_power_;
    private boolean has_steer_target_ ;
    private double target_angle_ ;
    private boolean has_drive_target_ ;
    private double target_speed_ ;
    private PIDCtrl angle_pid_ ;
    private PIDCtrl speed_pid_ ;
    private Speedometer linear_ ;
    private double ticks_ ;
    private double ticks_per_meter_ ;
    static private int logger_id_ = -1 ;

    private final String LinearSamplesName = "linear:samples" ;
    private final String TicksPerMeterName = "ticks_per_meter" ;

    public XeroSwerveModule(XeroRobot robot, XeroSwerveDriveSubsystem subsystem, String name, String config, String sname, boolean hwpid) throws BadParameterTypeException,
            MissingParameterException, EncoderConfigException, BadMotorRequestException, MotorRequestFailedException {

        ISettingsSupplier settings = subsystem.getRobot().getSettingsSupplier() ;
        MessageLogger logger = subsystem.getRobot().getMessageLogger() ;

        ticks_per_meter_ = subsystem.getSettingsValue(TicksPerMeterName).getDouble() ;

        robot_ = robot ;
        name_ = name ;

        steer_ = robot.getMotorFactory().createMotor(name + "-steer", config + ":hw:" + sname + ":steer");
        drive_ = robot.getMotorFactory().createMotor(name + "-drive", config + ":hw:" + sname + ":drive") ;
        encoder_ = new XeroEncoder(robot, config + ":hw:" + sname + ":encoder", true, null) ;

        drive_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Frequent, EncoderUpdateFrequency.Frequent);

        drive_power_ = 0.0;
        steer_power_ = 0.0;

        has_steer_target_ = false ;
        has_drive_target_ = false ;
        target_angle_ = 0.0 ;
        target_speed_ = 0.0 ;

        angle_pid_ = new PIDCtrl(settings, config + ":pid:" + sname + ":steer", true) ;

        if (hwpid) {
            speed_pid_ = null ;
            linear_ = null ;

            double p = settings.get(config + ":pid:" + sname + ":drive:kp").getDouble() ;
            double i = settings.get(config + ":pid:" + sname + ":drive:ki").getDouble() ;
            double d = settings.get(config + ":pid:" + sname + ":drive:kd").getDouble() ;
            double f = settings.get(config + ":pid:" + sname + ":drive:kf").getDouble() ;
            double outmax = settings.get(config + ":pid:" + sname + ":drive:max").getDouble() ;
            drive_.setPID(PidType.Velocity, p, i, d, f, outmax);

        }
        else {
            speed_pid_ = new PIDCtrl(settings, config + ":pid:" + sname + ":drive", false) ;

            int samples =  2 ;
            if (settings.isDefined(LinearSamplesName) && settings.get(LinearSamplesName).isInteger())
                samples = settings.get(LinearSamplesName).getInteger() ;
                
            linear_ = new Speedometer("linear", samples, false) ;
        }

        if (logger_id_ == -1)
            logger_id_ = logger.registerSubsystem("swervemodule") ;
    }

    public void run(double dt) throws BadMotorRequestException, MotorRequestFailedException {

        MessageLogger logger = robot_.getMessageLogger() ;
        logger.startMessage(MessageType.Debug, logger_id_) ;
        logger.add(name_) ;
                
        if (has_steer_target_)
        {
            double out = angle_pid_.getOutput(target_angle_, getAngle(), dt) ;

            logger.add(" [AnglePID") ;
            logger.add("target", target_angle_) ;
            logger.add("actual", getAngle()) ;
            logger.add("pidout", out) ;
            logger.add("]") ;
            steer_.set(out) ;

            steer_power_ = out ;
        }

        if (has_drive_target_ && speed_pid_ != null)
        {
            double out = speed_pid_.getOutput(target_speed_, getSpeed(), dt) ;
            logger.add(" [SpeedPID") ;
            logger.add("ticks", ticks_) ;
            logger.add("target", target_speed_) ;
            logger.add("actual", getSpeed()) ;
            logger.add("pidout", out) ;
            logger.add("]") ;
            drive_.set(out) ;

            drive_power_ = out ;
        }
        else
        {
            logger.add("[SpeedPID controller]");
        }
        logger.endMessage();
    }

    public void computeMyState(double dt) throws BadMotorRequestException {
        if (linear_ != null) {
            ticks_ = drive_.getPosition() ;
            linear_.update(dt, ticks_ / ticks_per_meter_) ;
        }
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getAngle())) ;
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), Rotation2d.fromDegrees(getAngle())) ;
    }

    public SwerveModuleState getModuleTarget() {
        return new SwerveModuleState(target_speed_, Rotation2d.fromDegrees(target_angle_)) ;
    }

    public double getRawSpeed() {
        double ret = 0.0 ;
        if (linear_ != null) {
           ret = linear_.getVelocity() ;
        }
        else {
            try {
                ret = drive_.getVelocity() ;
            } catch (BadMotorRequestException | MotorRequestFailedException e) {
                return 0.0 ;
            }
        }

        return ret ;
    }

    public double getDistance() {
        return linear_.getDistance() ;
    }

    public double getSpeed() {
        double ret = 0.0 ;

        if (linear_ != null) {
            ret = linear_.getVelocity() ;
        } else {
            try {
                double d = drive_.getVelocity() ;
                ret = d / ticks_per_meter_ * 10.0 ;
            }
            catch(Exception ex) {
                MessageLogger logger = robot_.getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("swervemodule: error reading velocity from drive motor - ").add(ex.getMessage()) ;
                logger.endMessage();
            }
        }
        return ret ;
    }

    public double getSpeedTarget() {
        return target_speed_;
    }

    public double getAngle() {
        return encoder_.getPosition() ;
    }

    public double getAngleTarget() {
        return target_angle_ ;
    }

    public void setNeutralMode(MotorController.NeutralMode mode) throws BadMotorRequestException, MotorRequestFailedException {
        steer_.setNeutralMode(mode);
        drive_.setNeutralMode(mode);
    }

    public double steerPower() {
        return steer_power_;
    }

    public double drivePower() {
        return drive_power_;
    }

    public void setSteerMotorPower(double p) throws BadMotorRequestException, MotorRequestFailedException {
        has_steer_target_ = false ;
        steer_power_ = p ;
        steer_.set(p) ;
    }

    public void setDriveMotorPower(double p) throws BadMotorRequestException, MotorRequestFailedException {
        has_drive_target_ = false ;
        drive_power_ = p ;
        drive_.set(p);
    }

    public void setPower(double steer, double drive) throws BadMotorRequestException, MotorRequestFailedException {
        setSteerMotorPower(steer);
        setDriveMotorPower(drive) ;
    }

    public double getTicks() {
        return ticks_ ;
    }

    public void setTargets(double angle, double speed) {

        has_steer_target_ = true ;
        has_drive_target_ = true ;

        double dist = Math.abs(XeroMath.normalizeAngleDegrees(angle - getAngle())) ;
        if (Math.abs(dist) > 90.0) {
            angle = XeroMath.normalizeAngleDegrees(angle + 180.0) ;
            speed = -speed ;
        }

        target_angle_ = angle ;
        target_speed_ = speed ;

        if (speed_pid_ == null) {
            try {
                //
                // The velocity of the talon fx device is measured in ticks per 100ms.  So, we need to conver
                // our meters per second to ticks per 100 ms
                //                
                double d = target_speed_ * ticks_per_meter_ / 10.0 ;  
                drive_.setTarget(d) ;
            }
            catch(Exception ex) {
                MessageLogger logger = robot_.getMessageLogger() ;
                logger.startMessage(MessageType.Error) ;
                logger.add("swervemodule: error writing velocity target to drive motor - ").add(ex.getMessage()) ;
                logger.endMessage();
            }            
        }
    }

    public void setAngle(double angle) {
        has_steer_target_ = true ;
        target_angle_ = angle ;
    }

    public String status() {
        String str = String.format("%.2f @ %.2f", getSpeed(), getAngle()) ;

        if (has_steer_target_ && has_drive_target_)
        {
            str += " t: " + String.format("%2f @ %2f", target_speed_, target_angle_) ;
        }

        return str ;
    }
}
