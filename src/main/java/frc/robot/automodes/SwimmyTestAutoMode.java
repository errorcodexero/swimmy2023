package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderHoldAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderMultiPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderVelocityAction;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveSetPoseAction;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveToPoseAction;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.base.subsystems.swerve.common.SwervePowerAngleAction;
import org.xero1425.base.subsystems.swerve.common.SwerveSpeedAngleAction;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.gpm.GPMCollectAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;

public class SwimmyTestAutoMode extends TestAutoMode {

    public SwimmyTestAutoMode(AutoController ctrl)
            throws InvalidActionRequest, Exception {
        super(ctrl, "Swimmy-Test-Mode");

        Swimmy2023RobotSubsystem robotsys = (Swimmy2023RobotSubsystem) ctrl.getRobot().getRobotSubsystem();
        SwerveBaseSubsystem swerve = (SwerveBaseSubsystem) robotsys.getDB();
        MotorEncoderSubsystem armLower = null ;
        MotorEncoderSubsystem armUpper = null ;
        MotorEncoderSubsystem grabberGrabMotor = null ;
        MotorEncoderSubsystem grabberSpinMotor = null ;

        double angles[] = new double[4] ;
        double powers[] = new double[4] ;

        GPMSubsystem gpm = robotsys.getGPM() ;
        ArmSubsystem arm = gpm.getArm() ;
        GrabberSubsystem grabber = gpm.getGrabber();

        armLower = arm.getLowerSubsystem() ;
        armUpper = arm.getUpperSubsystem() ;

        grabberGrabMotor = grabber.getGrabMotorSubsystem() ;
        grabberSpinMotor = grabber.getSpinMotorSubsystem() ;

        switch (getTestNumber()) {
            case 0:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run indefintely.  Action will
                // stop the plot after the default plot interval (four seconds)
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, getDouble("angle"), getDouble("power")), true) ;
                break;

            case 1:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run  until the duration has expired
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, getDouble("angle"), getDouble("power"), getDouble("duration")), true) ;
                break ;

            case 2:
                // Set the steering motor to the angle given, and the drive motor to the speed given.  Run indefintely.  Action will
                // stop the plot after the default plot interval (four seconds).  Since speed is given, the PID controller will try to
                // maintain the target speed
                addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"), getDouble("speed")), true) ;
                break;

            case 3:
                // Set the steering motor to the angle given, and the drive motor to the speed given.  Run  until the duration has expired.
                // Since speed is given, the PID controller will try to maintain the target speed
                addSubActionPair(swerve, new SwerveSpeedAngleAction(swerve, getDouble("angle"), getDouble("speed"), getDouble("duration")), true) ;
                break ;                

            case 4:
                // Run the path follower against the path given
                addSubActionPair(swerve, new SwerveHolonomicPathFollower(swerve, getString("name"), true, 3.0), true);
                break ;

            case 5:
                // Set the steering motor to the angle given, and the drive motor to the power given.  Run  until the duration has expired
                angles[0] = getDouble("angle");
                angles[1] = getDouble("angle");
                angles[2] = getDouble("angle");
                angles[3] = getDouble("angle");
                powers[0] = getDouble("power");
                powers[1] = getDouble("power");
                powers[2] = getDouble("power");
                powers[3] = getDouble("power");
                addSubActionPair(swerve, new SwervePowerAngleAction(swerve, angles, powers, getDouble("duration")), true) ;
                break ;
                
            case 10:
                addSubActionPair(armUpper, new MotorEncoderGotoAction(armUpper, getDouble("height"), true), true) ;
                addSubActionPair(armLower, new MotorEncoderPowerAction(armLower, getDouble("power"), getDouble("duration")), true) ;
                break ;

            case 11:
                addSubActionPair(armUpper, new MotorEncoderPowerAction(armUpper, getDouble("power"), getDouble("duration")), true) ;
                break ;

            case 12:
                addSubActionPair(armLower, new MotorEncoderVelocityAction(armLower, "follower", getDouble("velocity")), true) ;
                break ;

            case 13:
                addSubActionPair(armUpper, new MotorEncoderVelocityAction(armUpper, "follower", getDouble("velocity")), true) ;
                break ;

            case 14:
                addSubActionPair(armUpper, new MotorEncoderGotoAction(armUpper, getDouble("height"), true), true) ;
                addSubActionPair(armLower, new MotorEncoderGotoAction(armLower, getDouble("target"), true), true) ;
                break ;                

            case 15:
                addSubActionPair(armUpper, new MotorEncoderGotoAction(armUpper, getDouble("target"), true), true) ;
                break ;

            case 17:
                addSubActionPair(armUpper, new MotorEncoderGotoAction(armUpper, 20000, true), true) ;
                addAction(new DelayAction(getAutoController().getRobot(), 2.0)) ;
                addSubActionPair(armUpper, new MotorEncoderGotoAction(armUpper, 50000, true), true) ;
                addAction(new DelayAction(getAutoController().getRobot(), 2.0)) ;
                addSubActionPair(armUpper, new MotorEncoderGotoAction(armUpper, 30000, true), true) ;
                addAction(new DelayAction(getAutoController().getRobot(), 2.0)) ;
                break ;

            case 18:
                addSubActionPair(armUpper, new MotorEncoderGotoAction(armUpper, getDouble("upper"), true), false) ;
                addSubActionPair(armLower, new MotorEncoderGotoAction(armLower, getDouble("lower"), true), true) ;
                addAction(new DelayAction(getAutoController().getRobot(), 2.0)) ;
                addSubActionPair(armLower, new MotorEncoderGotoAction(armLower, 0, true), false) ;
                addAction(new DelayAction(getAutoController().getRobot(), 0.5)) ;
                addSubActionPair(armUpper, new MotorEncoderGotoAction(armUpper, 0, true), true) ;
                break ;

            case 19:
                if (RobotBase.isSimulation()) {
                    double [] mtimes = { 4.0, 4.0, 4.0, 4.0, 4.0 } ;
                    double [] mpowers = { 0.1, 0.3, 0.5, 0.7, 0.9 } ;
                    addSubActionPair(armLower, new MotorEncoderMultiPowerAction(armLower, mtimes, mpowers), true) ; ;
                }
                break ;

            case 20:
                addSubActionPair(grabberGrabMotor, new MotorEncoderPowerAction(grabberGrabMotor, getDouble("power"), getDouble("duration")), true) ;
                break ;

            case 21:
                addSubActionPair(grabberSpinMotor, new MotorEncoderPowerAction(grabberSpinMotor, getDouble("power"), getDouble("duration")), true) ;
                break ;

            case 22:
                addSubActionPair(grabberGrabMotor, new MotorEncoderGotoAction(grabberGrabMotor, getDouble("position"), true), true);
                break;

            case 23:
                addSubActionPair(grabberGrabMotor, new MotorEncoderHoldAction(grabberGrabMotor, getDouble("position")), true);
                break;

            case 24:
                addSubActionPair(grabberGrabMotor, new MotorEncoderHoldAction(grabberGrabMotor, 750), false);
                addAction(new DelayAction(getAutoController().getRobot(), 2.0));
                addSubActionPair(grabberGrabMotor, new MotorEncoderHoldAction(grabberGrabMotor, 50), false);
                addAction(new DelayAction(getAutoController().getRobot(), 2.0));
                addSubActionPair(grabberGrabMotor, new MotorEncoderHoldAction(grabberGrabMotor, 500), false);
                addAction(new DelayAction(getAutoController().getRobot(), 2.0));
                addSubActionPair(grabberGrabMotor, new MotorEncoderHoldAction(grabberGrabMotor, 0), false);
                addAction(new DelayAction(getAutoController().getRobot(), 2.0));
                break;                

            case 27:
                break ;

            case 28:
                break ;

            case 30:
                addSubActionPair(gpm, new GPMCollectAction(gpm), true);
                break ;

            case 99:
                {
                    Pose2d init = new Pose2d(getDouble("initx"), getDouble("inity"), Rotation2d.fromDegrees(getDouble("initangle")));
                    addSubActionPair(swerve, new SwerveDriveSetPoseAction(swerve, init), true);
                    addAction(new DelayAction(getAutoController().getRobot(), getDouble("delay"))) ;
                    Pose2d dest = new Pose2d(getDouble("x"), getDouble("y"), Rotation2d.fromDegrees(getDouble("angle")));
                    SwerveDriveToPoseAction act = new SwerveDriveToPoseAction(swerve, dest) ;
                    addSubActionPair(swerve, act, true);
                }
                break ;
        }
    }
    
}
