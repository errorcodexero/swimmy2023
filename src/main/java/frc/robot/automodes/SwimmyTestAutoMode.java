package frc.robot.automodes;

import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderMultiPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.base.subsystems.swerve.common.SwervePowerAngleAction;
import org.xero1425.base.subsystems.swerve.common.SwerveSpeedAngleAction;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GPMSubsystem;

public class SwimmyTestAutoMode extends TestAutoMode {

    private int which_ ;
    private String current_ ;

    public SwimmyTestAutoMode(AutoController ctrl) throws InvalidActionRequest, Exception {
        super(ctrl, "Swimmy-Test-Mode");

        which_ = -1 ;
        current_ = "" ;
    }

    @Override
    public String getCurrentModeName() {
        return current_ ;
    }

    @Override
    public String setTestNumber(int which) throws Exception {

        if (which_ == which)
            return current_ ;

        which_ = which ;

        clear() ;

        AutoController ctrl = getAutoController() ;
        Swimmy2023RobotSubsystem robotsys = (Swimmy2023RobotSubsystem) ctrl.getRobot().getRobotSubsystem();
        SwerveBaseSubsystem swerve = (SwerveBaseSubsystem) robotsys.getDB();
        MotorEncoderSubsystem armLower = null ;
        MotorEncoderSubsystem armUpper = null ;

        GPMSubsystem gpm = robotsys.getGPM() ;
        ArmSubsystem arm = gpm.getArm() ;

        if (RobotBase.isSimulation()) {
            armLower = arm.getMotorA() ;
            armUpper = arm.getMotorB() ;
        }

        switch (which_) {
            case 1:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run indefintely.  Action will
                // stop the plot after the default plot interval (four seconds)
                current_ = "Swerve Angle/Power" ;
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, getDouble("angle"), getDouble("power")), true) ;
                break;

            case 2:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run  until the duration has expired
                current_ = "Swerve Angle/Power/Duration" ;
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, getDouble("angle"), getDouble("power"), getDouble("duration")), true) ;
                break ;

            case 3:
                // Set the steering motor to the angle given, and the drive motor to the speed given.  Run indefintely.  Action will
                // stop the plot after the default plot interval (four seconds).  Since speed is given, the PID controller will try to
                // maintain the target speed
                current_ = "Swerve Angle/Speed" ;
                addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"), getDouble("speed")), true) ;
                break;

            case 4:
                // Set the steering motor to the angle given, and the drive motor to the speed given.  Run  until the duration has expired.
                // Since speed is given, the PID controller will try to maintain the target speed
                current_ = "Swerve Angle/Speed/Duration" ;
                addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"), getDouble("speed"), getDouble("duration")), true) ;
                break ;                

            case 5:
                // Run the path follower against the path given
                current_ = "Swerve Holonimic Path Follower" ;
                addSubActionPair(swerve, new SwerveHolonomicPathFollower(swerve, getString("name"), true), true);
                break ;
                
            case 10:
                current_ = "Lower Arm Power/Duration" ;
                if (RobotBase.isSimulation()) {
                    addSubActionPair(armLower, new MotorEncoderPowerAction(armLower, getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 11:
                current_ = "Upper Arm Power/Duration" ;
                if (RobotBase.isSimulation()) {
                    addSubActionPair(armUpper, new MotorEncoderPowerAction(armUpper, getDouble("power"), getDouble("duration")), true) ;
                }
                break ;

            case 12:
                current_ = "Lower Arm StairStep Power/Duration" ;
                if (RobotBase.isSimulation()) {
                    double [] mtimes = { 4.0, 4.0, 4.0, 4.0, 4.0 } ;
                    double [] mpowers = { 0.1, 0.3, 0.5, 0.7, 0.9 } ;
                    addSubActionPair(armLower, new MotorEncoderMultiPowerAction(armLower, mtimes, mpowers), true) ; ;
                }
                break ;

            default:
                current_ = "" ;
                break ;
        }

        return current_ ;
    }
}
