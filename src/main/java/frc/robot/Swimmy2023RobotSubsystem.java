package frc.robot;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.swerve.sdsswerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;

import frc.robot.subsystems.GPMSubsystem;
import frc.robot.subsystems.Swimmy2023OISubsystem;


public class Swimmy2023RobotSubsystem extends RobotSubsystem {
    private GPMSubsystem gpm_;
    private SDSSwerveDriveSubsystem db_;
    private Swimmy2023OISubsystem oi_;
    private LimeLightSubsystem limelight_;
    
    public Swimmy2023RobotSubsystem(XeroRobot robot) throws Exception {
        super(robot, "Swimmy2023RobotSubsystem") ;


        db_ = new SDSSwerveDriveSubsystem(this, "db" );
        addChild(db_);

        oi_ = new Swimmy2023OISubsystem(this, db_);
        addChild(oi_);

        limelight_ = new LimeLightSubsystem(this, "limelight");
        addChild(limelight_);
        
        gpm_ = new GPMSubsystem(this);
        addChild(gpm_);


        

        
    }

}