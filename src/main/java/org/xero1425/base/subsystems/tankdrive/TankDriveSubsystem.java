package org.xero1425.base.subsystems.tankdrive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import org.xero1425.base.LoopType;
import org.xero1425.base.motors.BadMotorRequestException;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.MotorRequestFailedException;
import org.xero1425.base.motors.MotorController.EncoderUpdateFrequency;
import org.xero1425.base.subsystems.DriveBaseSubsystem;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.Speedometer;

/// \file

/// \brief The tankdrive subsystem for driving a robot using a tank drive type drivebase.
/// 
public class TankDriveSubsystem extends DriveBaseSubsystem {

    static private final String CurrentLimitName = "current_limit" ;

    private DifferentialDriveOdometry tracker_ ;
    private DifferentialDriveKinematics kinematic_ ;

    private double left_power_ ;
    private double right_power_ ;
    private int ticks_left_ ;
    private int ticks_right_ ;
    private double dist_l_ ;
    private double dist_r_ ;
    private double last_dist_l_ ;
    private double last_dist_r_ ;
    private double left_inches_per_tick_ ;
    private double right_inches_per_tick_ ;
    private double total_angle_ ;
    private MotorController.NeutralMode automode_neutral_ ;
    private MotorController.NeutralMode teleop_neutral_ ;
    private MotorController.NeutralMode disabled_neutral_ ;

    private Speedometer left_linear_ ;
    private Speedometer right_linear_ ;

    private double rotational_velocity_ ;
    private Pose2d last_pose_ ;

    private double teleop_ramp_rate_ ;
    private double auto_ramp_rate_ ;

    private MotorController left_motors_ ;
    private MotorController right_motors_ ;
    private Encoder left_encoder_ ;
    private Encoder right_encoder_ ;

    private boolean recording_ ;

    /// \brief create a new tankdrive subsystem
    /// \param parent the parent subsystem
    /// \param name the name of the subsystem
    /// \param config the string prefix to use when searching for settings file entries
    public TankDriveSubsystem(Subsystem parent, String name, String config) throws Exception {
        super(parent, name);

        ISettingsSupplier settings = getRobot().getSettingsSupplier() ;

        recording_ = false ;

        double width = getSettingsValue("width").getDouble() ;

        tracker_ = new DifferentialDriveOdometry(getAngle(), 0.76, 0.76) ;
        kinematic_ = new DifferentialDriveKinematics(width) ;

        dist_l_ = 0.0;
        dist_r_ = 0.0;
        last_dist_l_ = 0.0 ;
        last_dist_r_ = 0.0 ;

        left_inches_per_tick_ = getSettingsValue("ticks_per_meter").getDouble();
        right_inches_per_tick_ = left_inches_per_tick_;

        int linearsamples = 2 ;
        String linear = "linearsamples" ;

        if (settings.isDefined(linear) && settings.get(linear).isInteger()) {
            linearsamples = settings.get(linear).getInteger() ;
        }

        left_linear_ = new Speedometer("left", linearsamples, false);
        right_linear_ = new Speedometer("right", linearsamples, false);

        automode_neutral_ = MotorController.NeutralMode.Brake;
        teleop_neutral_ = MotorController.NeutralMode.Brake;
        disabled_neutral_ = MotorController.NeutralMode.Coast;

        teleop_ramp_rate_ = 0.0 ;
        auto_ramp_rate_ = 0.0 ;

        attachHardware();

        last_pose_ = new Pose2d() ;
    }

    /// \brief set the open loop ramp rate for the tank drive motors
    /// \param teleop the ramp rate for teleop mode
    /// \param auto the ramp rate for auto mode
    public void setOpenLoopRampRates(double teleop, double auto) {
        teleop_ramp_rate_ = teleop ;
        auto_ramp_rate_ = auto ;
    }

    /// \brief sets the recording flag for the drive base.  
    /// When the recording flag is set, the drivebase subsystem writes the pose for the drivebase
    /// into the network tables at the location /Smartdashboard/db-trk-t, /Smartdashboard/db-trk-x, 
    /// /Smartdashboard/db-trk-y,and /Smartdashboard/db-trk-a.  The time is relative to when setRecording
    /// was called with a value of true.  The angle is in degrees.
    public void setRecording(boolean v) {
        recording_ = v ;
    }

    // /// \brief returns the width of the robot
    // /// This width is the track width which is basically from the center of the left 
    // /// wheels to the center of the right wheels.
    // /// \returns the width of the robot
    // public double getWidth() {
    //     return tracker_.getWidth() ;
    // }

    // /// \brief returns the scrub value for the robot
    // /// \returns the scrub value for the robot
    // public double getScrub() {
    //     return tracker_.getScrub() ;
    // }

    /// \brief returns the distance traveled by the left sdie of the robot
    /// \returns the distance traveled by the left sdie of the robot     
    public double getLeftDistance() {
        return dist_l_;
    }

    /// \brief returns the distance traveled by the right sdie of the robot
    /// \returns the distance traveled by the right sdie of the robot 
    public double getRightDistance() {
        return dist_r_;
    }

    /// \brief returns the distance traveled by the robot
    /// \returns the distance traveled by the robot
    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    /// \brief returns the velocity of the left side of the robot
    /// \returns the velocity of the left side of the robot    
    public double getLeftVelocity() {
        return left_linear_.getVelocity() ;
    }

    /// \brief returns the velocity of the right side of the robot
    /// \returns the velocity of the right side of the robot    
    public double getRightVelocity() {
        return right_linear_.getVelocity() ;
    }

    /// \brief returns the velocity of the robot
    /// \returns the velocity of the robot
    public double getVelocity() {
        return (left_linear_.getVelocity() + right_linear_.getVelocity()) / 2.0 ;
    }

    public double getRotationalVelocity() {
        return rotational_velocity_ ;
    }

    /// \brief returns the acceleration of the left side of the robot
    /// \returns the acceleration of the left side of the robot
    public double getLeftAcceleration() {
        return left_linear_.getAcceleration() ;
    }

    /// \brief returns the acceleration of the right side of the robot
    /// \returns the acceleration of the right side of the robot
    public double getRightAcceleration() {
        return right_linear_.getAcceleration() ;
    }    

    /// \brief returns the acceleration of the robot
    /// \returns the acceleration of the robot
    public double getAcceleration() {
        return (left_linear_.getAcceleration() + right_linear_.getAcceleration()) / 2.0 ;
    }

    /// \brief returns the left side ticks value from the encoder
    /// \returns the left side ticks value from the encoder
    public int getLeftTick() {
        return ticks_left_ ;
    }

    /// \brief returns the right side ticks value from the encoder
    /// \returns the right side ticks value from the encoder
    public int getRightTick() {
        return ticks_right_ ;
    }

    /// \brief returns the current angle in degrees of the robot
    /// \returns the current angle of the robot
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro().getYaw()) ;
    }

    public double getWidth() {
        return kinematic_.trackWidthMeters ;
    }

    /// \brief returns the net total angle in degrees the robot has rotated since the drivebase was initialization
    /// If the robot has rotated in place two complete revolutions, this would return 720 degrees
    /// \returns the net total angle the robot has rotated since the drivebase was initialization
    public double getTotalAngle() {
        return total_angle_ ;
    }

    /// \brief This method is called when the robot enters the disabled state.
    /// It is used in this subsystem to set the neutral mode of the drivebase
    /// motors.
    public void reset() {
        super.reset();

        try {
            left_motors_.setNeutralMode(disabled_neutral_);
            right_motors_.setNeutralMode(disabled_neutral_);
        } catch (Exception ex) {
        }
    }

    /// \brief This method is called when the robot enters one of its specifc modes.
    /// The modes are Autonomous, Teleop, Test, or Disabled.  It is used to set the
    /// neutral mode specifically for the robot mode.
    public void init(LoopType ltype) {
        super.init(ltype);

        try {
            switch (ltype) {
            case Autonomous:
                left_motors_.setNeutralMode(automode_neutral_);
                right_motors_.setNeutralMode(automode_neutral_);
                left_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Frequent, EncoderUpdateFrequency.Infrequent);
                right_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Frequent, EncoderUpdateFrequency.Infrequent);
                left_motors_.setOpenLoopRampRate(auto_ramp_rate_) ;
                right_motors_.setOpenLoopRampRate(auto_ramp_rate_) ;    
                break;

            case Teleop:
                left_motors_.setNeutralMode(teleop_neutral_);
                right_motors_.setNeutralMode(teleop_neutral_);
                left_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Infrequent, EncoderUpdateFrequency.Infrequent);
                right_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Infrequent, EncoderUpdateFrequency.Infrequent);
                left_motors_.setOpenLoopRampRate(teleop_ramp_rate_) ;
                right_motors_.setOpenLoopRampRate(teleop_ramp_rate_) ;
                break;

            case Test:
                left_motors_.setNeutralMode(disabled_neutral_);
                right_motors_.setNeutralMode(disabled_neutral_);
                left_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Infrequent, EncoderUpdateFrequency.Infrequent);
                right_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Infrequent, EncoderUpdateFrequency.Infrequent);
                break;

            case Disabled:
                left_motors_.setNeutralMode(disabled_neutral_);
                right_motors_.setNeutralMode(disabled_neutral_);      
                left_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Infrequent, EncoderUpdateFrequency.Infrequent);
                right_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Infrequent, EncoderUpdateFrequency.Infrequent) ;
                break ;
            }
        } catch (Exception ex) {
        }
    }

    /// \brief set the pose (x and y location plus heading) of the robot
    /// \param pose the pose for the robot
    public void setPose(Pose2d pose) {
        tracker_.resetPosition(getAngle(), 0.0, 0.0, pose) ;
        try {
            left_motors_.resetEncoder();
            right_motors_.resetEncoder();
        }
        catch(BadMotorRequestException ex) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("error occurred setting the tank drive pose - " + ex.getMessage()) ;
        }
    }

    /// \brief get the pose for the robot
    /// This method returns the current pose based on the position tracker
    /// assigned to the drivebase.
    /// \returns the current pose for the robot
    public Pose2d getPose() {
        return tracker_.getPoseMeters() ;
    }

    /// \brief compute the state of the drivebase.
    /// This method reads the encoders for the left and right sides of the drivebase
    /// and reads the gyro as well.  These three are used to compute the current state
    /// of the robot and to update the position tracker for the robot.
    public void computeMyState() {
        MessageLogger logger = getRobot().getMessageLogger() ;

        try {
            if (left_motors_.hasPosition() && right_motors_.hasPosition()) {
                ticks_left_ = (int)left_motors_.getPosition();
                ticks_right_ = (int)right_motors_.getPosition();
            }
            else {
                ticks_left_ = left_encoder_.get() ;
                ticks_right_ = right_encoder_.get()  ;
            }

            dist_l_ = ticks_left_ * left_inches_per_tick_;
            dist_r_ = ticks_right_ * right_inches_per_tick_;

            total_angle_ = gyro().getAngle() ;

            tracker_.update(getAngle(), dist_l_ - last_dist_l_, dist_r_ - last_dist_r_);
            left_linear_.update(getRobot().getDeltaTime(), getLeftDistance());
            right_linear_.update(getRobot().getDeltaTime(), getRightDistance());

            last_dist_l_ = dist_l_ ;
            last_dist_r_ = dist_r_ ;

            putDashboard("ldist", DisplayType.Verbose, dist_l_);
            putDashboard("rdist", DisplayType.Verbose, dist_r_);

        } catch (Exception ex) {
            //
            // This should never happen
            //
        }

        if (recording_) {
            logger.startMessage(MessageType.Info, getLoggerID()) ;
            logger.add("TankDrive: ") ;
            logger.add("db-trk-t", getRobot().getTime()) ;
            logger.add("db-trk-x", tracker_.getPoseMeters().getX()) ;
            logger.add("db-trk-y", tracker_.getPoseMeters().getY()) ;
            logger.add("db-trk-a", tracker_.getPoseMeters().getRotation().getDegrees()) ;
            logger.endMessage();
        }

        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add("Power: ") ;
        logger.add("left", left_power_).add("right", right_power_).endMessage();

        Pose2d p = getPose() ;
        rotational_velocity_ = (p.getRotation().getDegrees() - last_pose_.getRotation().getDegrees()) / getRobot().getDeltaTime() ;
        last_pose_ = p ;
    }

    /// \brief set the power for the tank drive
    /// \param v the velocity of the left and right sides of the drivebase
    protected void setPower(DifferentialDriveWheelSpeeds v) {
        setPower(v.leftMetersPerSecond, v.rightMetersPerSecond) ;
    }

    /// \brief set the power for the tank drive
    /// \param left the power for the left side of the drivebase
    /// \param right the power for the right side of the drivebase
    public void setPower(double left, double right) {
        MessageLogger logger = getRobot().getMessageLogger() ;

        left_power_ = left ;
        right_power_ = right ;

        try {
            left_motors_.set(left_power_) ;
            right_motors_.set(right_power_) ;
        }
        catch(BadMotorRequestException|MotorRequestFailedException ex) {
            logger.startMessage(MessageType.Error) ;
            logger.add("subsystem ").addQuoted(getName()).add(": cannot set power -").add(ex.getMessage()).endMessage();
        }
    }

    private void attachHardware() throws BadMotorRequestException, MissingParameterException, BadParameterTypeException {
        MessageLogger logger = getRobot().getMessageLogger() ;

        left_motors_ = getRobot().getMotorFactory().createMotor("TankDriveLeft", "subsystems:" + getName() + ":hw:left:motors") ;
        right_motors_ = getRobot().getMotorFactory().createMotor("TankDriveRight", "subsystems:" + getName() + ":hw:right:motors") ;

        if (isSettingDefined(CurrentLimitName)) {
            double limit = getSettingsValue(CurrentLimitName).getDouble() ;
            left_motors_.setCurrentLimit(limit) ;
            right_motors_.setCurrentLimit(limit);

            logger.startMessage(MessageType.Info) ;
            logger.add("TankDrive: motors current limited to " + limit + " Amps") ;
            logger.endMessage();
        }

        if (left_motors_ == null || right_motors_ == null) {

            logger.startMessage(MessageType.Error, getLoggerID()) ;
            if (left_motors_ == null)
                logger.add("could not create left motors, ") ;
            if (right_motors_ == null)
                logger.add("could not create right motors") ;            
            logger.endMessage();
        }


        if (!left_motors_.hasPosition() || !right_motors_.hasPosition()) {
            int p1, p2 ;

            p1 = getSettingsValue("hw:left:encoders:1").getInteger() ;
            p2 = getSettingsValue("hw:left:encoders:2").getInteger() ;
            left_encoder_ = new Encoder(p1, p2) ;


            p1 = getSettingsValue("hw:right:encoders:1").getInteger() ;
            p2 = getSettingsValue("hw:right:encoders:2").getInteger() ;
            right_encoder_ = new Encoder(p1, p2) ;

            logger.startMessage(MessageType.Info, getLoggerID()) ;
            logger.add("TankDrive FPGA encoder indexes: ") ;
            logger.add("left", left_encoder_.getFPGAIndex()) ;
            logger.add("right", right_encoder_.getFPGAIndex()) ;
            logger.endMessage();
        }

        if (left_motors_.hasPosition() && right_motors_.hasPosition()) {
            left_motors_.resetEncoder(); 
            right_motors_.resetEncoder();
            
            left_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Frequent, EncoderUpdateFrequency.Infrequent);
            right_motors_.setEncoderUpdateFrequncy(EncoderUpdateFrequency.Frequent, EncoderUpdateFrequency.Infrequent);
        }
    }
}
