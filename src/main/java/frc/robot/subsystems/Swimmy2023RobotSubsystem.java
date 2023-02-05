package frc.robot.subsystems;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.sdsswerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

import frc.robot.subsystems.gpm.GPMSubsystem;


public class Swimmy2023RobotSubsystem extends RobotSubsystem {
    private GPMSubsystem gpm_;
    private SDSSwerveDriveSubsystem db_;
    private Swimmy2023OISubsystem oi_;
    private LimeLightSubsystem limelight_;
    
    public Swimmy2023RobotSubsystem(XeroRobot robot) throws Exception {
        super(robot, "Swimmy2023RobotSubsystem") ;

        db_ = new SDSSwerveDriveSubsystem(this, "swerve" );
        addChild(db_);

        oi_ = new Swimmy2023OISubsystem(this, db_);
        addChild(oi_);

        // limelight_ = new LimeLightSubsystem(this, "limelight");
        // addChild(limelight_);
        
        gpm_ = new GPMSubsystem(this);
        addChild(gpm_);        
    }

    public GPMSubsystem getGPM() {
        return gpm_ ;
    }

    public SwerveBaseSubsystem getSwerve() {
        return db_ ;
    }

    public Swimmy2023OISubsystem getOI() {
        return oi_ ;
    }

    public LimeLightSubsystem getLimeLight() {
        return limelight_ ;
    }
}