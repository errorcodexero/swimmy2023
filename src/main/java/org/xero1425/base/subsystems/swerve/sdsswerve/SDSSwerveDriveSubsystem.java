package org.xero1425.base.subsystems.swerve.sdsswerve;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;

import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SDSSwerveDriveSubsystem extends SwerveBaseSubsystem {

    enum Mode {
        RawPower,
        RawSpeed,
        Chassis
    } ;

    private final SwerveModule fl_ ;
    private final SwerveModule fr_ ;
    private final SwerveModule bl_ ;
    private final SwerveModule br_ ;

    private PIDCtrl[] pid_ctrls_ ;

    private ChassisSpeeds chassis_speed_ ;

    private Mode mode_ ;
    private double [] speeds_ ;
    private double [] powers_ ;
    private double [] angles_ ;

    private double nominal_voltage_ ;

    public SDSSwerveDriveSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name) ;

        speeds_ = new double[4] ;
        powers_ = new double[4] ;
        angles_ = new double[4] ;

        pid_ctrls_ = new PIDCtrl[4] ;
        pid_ctrls_[FL] = createPIDCtrl("fl") ;
        pid_ctrls_[FR] = createPIDCtrl("fr") ;
        pid_ctrls_[BL] = createPIDCtrl("bl") ;
        pid_ctrls_[BR] = createPIDCtrl("br") ;

        if (DriverStation.isFMSAttached()) {
            fl_ = createSwerveModule("fl", -1) ;
            fr_ = createSwerveModule("fr", -1) ;
            bl_ = createSwerveModule("bl", -1) ;
            br_ = createSwerveModule("br", -1) ;
        }
        else {
            fl_ = createSwerveModule("fl", 0) ;
            fr_ = createSwerveModule("fr", 2) ;
            bl_ = createSwerveModule("bl", 4) ;
            br_ = createSwerveModule("br", 6) ;
        }

        createOdometry(); 
    }


    public SwerveModuleState getModuleState(int which) {
        SwerveModuleState st = null ;

        switch(which) {
            case FL:
                st = new SwerveModuleState(fl_.getDriveVelocity(), new Rotation2d(fl_.getSteerAngle())) ;
                break ;

            case FR:
                st = new SwerveModuleState(fr_.getDriveVelocity(), new Rotation2d(fr_.getSteerAngle())) ;
                break ;
                
            case BL:
                st = new SwerveModuleState(bl_.getDriveVelocity(), new Rotation2d(bl_.getSteerAngle())) ;
                break ;
                
            case BR:
                st = new SwerveModuleState(br_.getDriveVelocity(), new Rotation2d(br_.getSteerAngle())) ;
                break ;
        }

        return st ;
    }

    public SwerveModulePosition getModulePosition(int which) {
        SwerveModulePosition st = null ;

        switch(which) {
            case FL:
                st = new SwerveModulePosition(fl_.getDriveDistance(), new Rotation2d(fl_.getSteerAngle())) ;
                break ;

            case FR:
                st = new SwerveModulePosition(fr_.getDriveDistance(), new Rotation2d(fr_.getSteerAngle())) ;
                break ;
                
            case BL:
                st = new SwerveModulePosition(bl_.getDriveDistance(), new Rotation2d(bl_.getSteerAngle())) ;
                break ;
                
            case BR:
                st = new SwerveModulePosition(br_.getDriveDistance(), new Rotation2d(br_.getSteerAngle())) ;
                break ;
        }

        return st ;
    }

    public SwerveModuleState getModuleTarget(int which) {
        SwerveModuleState st = null ;

        st = new SwerveModuleState(speeds_[which], Rotation2d.fromDegrees(angles_[which])) ;
        return st ;
    }


    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(gyro().getYaw()) ;
    }

    public void drive(ChassisSpeeds speed) {
        chassis_speed_ = speed ;     
        mode_ = Mode.Chassis ;
    }

    @Override
    public void setRawTargets(boolean power, double [] angles, double [] speeds_powers)  {
        angles_ = angles.clone() ;
        if (power) {
            mode_ = Mode.RawPower ;
            powers_ = speeds_powers.clone() ;
        }
        else {
            mode_ = Mode.RawSpeed ;
            speeds_ = speeds_powers.clone() ;
        }
    }

    @Override
    public void computeMyState() throws Exception {
        super.computeMyState();

        if (getRobot().isDisabled())
        {
            fl_.set(0.0, 0.0) ;
            fr_.set(0.0, 0.0) ;
            bl_.set(0.0, 0.0) ;
            br_.set(0.0, 0.0) ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        //
        // Just in case, paranoid code.  Be sure the arrays we are intersted in are there.  SHould be set
        // in the constructor and in any setter that provides new values
        //
        if (angles_ == null || angles_.length != 4)
            angles_ = new double[4] ;

        if (speeds_ == null || speeds_.length != 4)
            speeds_ = new double[4] ;

        if (powers_ == null || powers_.length != 4)
            powers_ = new double[4] ;   

        if (mode_ == Mode.Chassis) {

            // Convert chassis speeds to module speeds and angles
            SwerveModuleState[] states = getKinematics().toSwerveModuleStates(chassis_speed_);
            
            angles_[FL] = states[FL].angle.getDegrees() ;
            speeds_[FL] = states[FL].speedMetersPerSecond ;
            angles_[FR] = states[FR].angle.getDegrees() ;
            speeds_[FR] = states[FR].speedMetersPerSecond ;
            angles_[BL] = states[BL].angle.getDegrees() ;
            speeds_[BL] = states[BL].speedMetersPerSecond ;
            angles_[BR] = states[BR].angle.getDegrees() ;
            speeds_[BR] = states[BR].speedMetersPerSecond ;                                    
        }

        if (mode_ == Mode.Chassis || mode_ == Mode.RawSpeed)
        {
            powers_[FL] = pid_ctrls_[FL].getOutput(speeds_[FL], getModuleState(FL).speedMetersPerSecond, getRobot().getDeltaTime()) ;
            powers_[FR] = pid_ctrls_[FR].getOutput(speeds_[FR], getModuleState(FR).speedMetersPerSecond, getRobot().getDeltaTime()) ;
            powers_[BL] = pid_ctrls_[BL].getOutput(speeds_[BL], getModuleState(BL).speedMetersPerSecond, getRobot().getDeltaTime()) ;
            powers_[BR] = pid_ctrls_[BR].getOutput(speeds_[BR], getModuleState(BR).speedMetersPerSecond, getRobot().getDeltaTime()) ;
        }

        fl_.set(powers_[FL] * nominal_voltage_, Math.toRadians(angles_[FL])) ;
        fr_.set(powers_[FR] * nominal_voltage_, Math.toRadians(angles_[FR])) ;
        bl_.set(powers_[BL] * nominal_voltage_, Math.toRadians(angles_[BL])) ;
        br_.set(powers_[BR] * nominal_voltage_, Math.toRadians(angles_[BR])) ;                      
    }

    
    private SwerveModule createSwerveModule(String which, int pos) throws BadParameterTypeException, MissingParameterException {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        SwerveModule ret ;

        int drive = getSettingsValue("hw:" + which + ":drive:canid").getInteger() ;
        String drivebus = getSettingsValue("hw:" + which + ":drive:bus").getString() ;
        int steer = getSettingsValue("hw:" + which + ":steer:canid").getInteger() ;
        String steerbus = getSettingsValue("hw:" + which + ":steer:bus").getString() ;
        int encoder = getSettingsValue("hw:" + which + ":encoder:canid").getInteger() ;
        String encoderbus = getSettingsValue("hw:" + which + ":encoder:bus").getString() ;
        double offset = Math.toRadians(getSettingsValue("hw:" + which + ":encoder:offset").getDouble()) ;

        if (pos == -1) {
            ret = new MkSwerveModuleBuilder()
            .withGearRatio(SdsModuleConfigurations.MK4I_L2)
            .withDriveMotor(MotorType.FALCON, drive, drivebus)
            .withSteerMotor(MotorType.FALCON, steer, steerbus)
            .withSteerEncoderPort(encoder, encoderbus)
            .withSteerOffset(offset)
            .build() ;
        }
        else {
            ret = new MkSwerveModuleBuilder()
                    .withLayout(shuffleboardTab.getLayout("FL", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                    .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                    .withDriveMotor(MotorType.FALCON, drive, drivebus)
                    .withSteerMotor(MotorType.FALCON, steer, steerbus)
                    .withSteerEncoderPort(encoder, encoderbus)
                    .withSteerOffset(offset)
                    .build() ;
        }
        return ret ;
    }

    private PIDCtrl createPIDCtrl(String name) throws MissingParameterException, BadParameterTypeException {
        String pidname = "subsystems:" + getName() + ":pids:" + name ;
        return new PIDCtrl(getRobot().getSettingsSupplier(), pidname, false) ;
    }
}
