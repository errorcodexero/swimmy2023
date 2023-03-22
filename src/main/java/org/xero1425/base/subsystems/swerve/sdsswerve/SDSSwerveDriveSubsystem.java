package org.xero1425.base.subsystems.swerve.sdsswerve;

import org.xero1425.swervelib.Mk4ModuleConfiguration;
import org.xero1425.swervelib.Mk4iSwerveModuleHelper;
import org.xero1425.swervelib.SDSModuleGlobalConfig;
import org.xero1425.swervelib.SwerveModule;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.PIDCtrl;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
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

    private boolean module_encoders_inited_ ;
    private XeroTimer module_init_timer_ ;
    
    public SDSSwerveDriveSubsystem(Subsystem parent, String name) throws Exception {
        super(parent, name) ;

        ShuffleboardLayout lay ;
        int drive, steer, encoder ;
        double offset ;
        Mk4ModuleConfiguration config = new Mk4ModuleConfiguration() ;

        String bus = getSettingsValue("hw:bus").getString() ;
        SDSModuleGlobalConfig.setCanBus(bus) ;

        speeds_ = new double[4] ;
        powers_ = new double[4] ;
        angles_ = new double[4] ;

        pid_ctrls_ = new PIDCtrl[4] ;
        pid_ctrls_[FL] = createPIDCtrl("fl") ;
        pid_ctrls_[FR] = createPIDCtrl("fr") ;
        pid_ctrls_[BL] = createPIDCtrl("bl") ;
        pid_ctrls_[BR] = createPIDCtrl("br") ;

        if (isSettingDefined("electrical:drive-current-limit")) {
            config.setDriveCurrentLimit(getSettingsValue("electrical:drive-current-limit").getDouble()) ;
        }

        if (isSettingDefined("electrical:steer-current-limit")) {
            config.setSteerCurrentLimit(getSettingsValue("electrical:steer-current-limit").getDouble()) ;
        }

        if (isSettingDefined("electrical:nominal-voltage")) {
            nominal_voltage_ = getSettingsValue("electrical:nominal-voltage").getDouble() ;
            config.setNominalVoltage(nominal_voltage_) ;
        }
        else {
            nominal_voltage_ = 12.0 ;
        }

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        lay = shuffleboardTab.getLayout("FLModule", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0) ;
        drive = getSettingsValue("hw:fl:drive:canid").getInteger() ;
        steer = getSettingsValue("hw:fl:steer:canid").getInteger() ;
        encoder = getSettingsValue("hw:fl:encoder:canid").getInteger() ;
        offset = Math.toRadians(getSettingsValue("hw:fl:encoder:offset").getDouble()) ;

        if (isVerbose())
            fl_ = Mk4iSwerveModuleHelper.createFalcon500(lay, config, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;
        else
            fl_ = Mk4iSwerveModuleHelper.createFalcon500(config, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;

        lay = shuffleboardTab.getLayout("FRModule", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0) ;
        drive = getSettingsValue("hw:fr:drive:canid").getInteger() ;
        steer = getSettingsValue("hw:fr:steer:canid").getInteger() ;
        encoder = getSettingsValue("hw:fr:encoder:canid").getInteger() ;
        offset = Math.toRadians(getSettingsValue("hw:fr:encoder:offset").getDouble()) ;

        if (isVerbose())
            fr_ = Mk4iSwerveModuleHelper.createFalcon500(lay, config, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;
        else
            fr_ = Mk4iSwerveModuleHelper.createFalcon500(config, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;

        lay = shuffleboardTab.getLayout("BLModule", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0) ;
        drive = getSettingsValue("hw:bl:drive:canid").getInteger() ;
        steer = getSettingsValue("hw:bl:steer:canid").getInteger() ;
        encoder = getSettingsValue("hw:bl:encoder:canid").getInteger() ;
        offset = Math.toRadians(getSettingsValue("hw:bl:encoder:offset").getDouble()) ;
        if (isVerbose())
            bl_ = Mk4iSwerveModuleHelper.createFalcon500(lay, config, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;
        else
            bl_ = Mk4iSwerveModuleHelper.createFalcon500(config, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;        

        lay = shuffleboardTab.getLayout("BRModule", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0) ;
        drive = getSettingsValue("hw:br:drive:canid").getInteger() ;
        steer = getSettingsValue("hw:br:steer:canid").getInteger() ;
        encoder = getSettingsValue("hw:br:encoder:canid").getInteger() ;
        offset = Math.toRadians(getSettingsValue("hw:br:encoder:offset").getDouble()) ;
        if (isVerbose())
            br_ = Mk4iSwerveModuleHelper.createFalcon500(lay, config, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;
        else
            br_ = Mk4iSwerveModuleHelper.createFalcon500(config, Mk4iSwerveModuleHelper.GearRatio.L2, drive, steer, encoder, offset) ;        

        createOdometry(); 

        module_init_timer_ = new XeroTimer(parent.getRobot(), "swerve-init", 15.0);
        module_encoders_inited_ = true ;
        module_init_timer_.start() ;
    }

    private PIDCtrl createPIDCtrl(String name) throws MissingParameterException, BadParameterTypeException {
        String pidname = "subsystems:" + getName() + ":pids:" + name ;
        return new PIDCtrl(getRobot().getSettingsSupplier(), pidname, false) ;
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
                st = new SwerveModulePosition(fl_.getDistance(), new Rotation2d(fl_.getSteerAngle())) ;
                break ;

            case FR:
                st = new SwerveModulePosition(fr_.getDistance(), new Rotation2d(fr_.getSteerAngle())) ;
                break ;
                
            case BL:
                st = new SwerveModulePosition(bl_.getDistance(), new Rotation2d(bl_.getSteerAngle())) ;
                break ;
                
            case BR:
                st = new SwerveModulePosition(br_.getDistance(), new Rotation2d(br_.getSteerAngle())) ;
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

        // fl_.heartBeat(getRobot().getMessageLogger(), "FL");
        // fr_.heartBeat(getRobot().getMessageLogger(), "FR");
        // bl_.heartBeat(getRobot().getMessageLogger(), "BL");
        // br_.heartBeat(getRobot().getMessageLogger(), "BR");

        if (module_init_timer_.isExpired() && !module_encoders_inited_) {
            fl_.synchronizeEncoders(getRobot().getMessageLogger(), "FL");
            fr_.synchronizeEncoders(getRobot().getMessageLogger(), "FR");
            bl_.synchronizeEncoders(getRobot().getMessageLogger(), "BL");
            br_.synchronizeEncoders(getRobot().getMessageLogger(), "BR");

            module_encoders_inited_ = true ;
        }

        if (getRobot().isDisabled()) {
            fl_.set(0.0, 0.0);
            fr_.set(0.0, 0.0);
            bl_.set(0.0, 0.0);
            br_.set(0.0, 0.0);
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

        if (module_encoders_inited_) {
            fl_.set(powers_[FL] * nominal_voltage_, Math.toRadians(angles_[FL])) ;
            fr_.set(powers_[FR] * nominal_voltage_, Math.toRadians(angles_[FR])) ;
            bl_.set(powers_[BL] * nominal_voltage_, Math.toRadians(angles_[BL])) ;
            br_.set(powers_[BR] * nominal_voltage_, Math.toRadians(angles_[BR])) ;

            // MessageLogger logger = getRobot().getMessageLogger();
            // logger.startMessage(MessageType.Info);
            // logger.add("swerve driving: ") ;
            // logger.add("fl", angles_[FL]);
            // logger.add("fr", angles_[FR]);
            // logger.add("bl", angles_[BL]);
            // logger.add("br", angles_[BR]);
            // logger.endMessage();
        }
        else {
            MessageLogger logger = getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info);
            logger.add("SDSSwerveDriveSubsystem: waiting on timer to init angles from encoders") ;
            logger.endMessage();
        }
    }
}
