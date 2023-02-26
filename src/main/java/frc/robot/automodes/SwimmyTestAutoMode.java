package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderHoldAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveAlignDriveBaseAction;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.base.subsystems.swerve.common.SwervePowerAngleAction;
import org.xero1425.base.subsystems.swerve.common.SwerveSpeedAngleAction;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;
import org.xero1425.misc.SCurveConfig;
import org.xero1425.misc.TrapezoidalProfileConfig;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.arm.ArmGotoAction;
import frc.robot.subsystems.arm.ArmStaggeredGotoAction;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.gpm.GPMCollectAction;
import frc.robot.subsystems.gpm.GPMPlaceAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.grabber.GrabberGrabGampieceAction;
import frc.robot.subsystems.grabber.GrabberStartCollectAction;
import frc.robot.subsystems.grabber.GrabberStopCollectAction;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.swerve.SwerveAlignRobotAction;
import frc.robot.subsystems.toplevel.RobotOperation;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class SwimmyTestAutoMode extends TestAutoMode {

    private Pose2d initial_pose_ = new Pose2d();

    public SwimmyTestAutoMode(AutoController ctrl)
            throws InvalidActionRequest, Exception {
        super(ctrl, "Swimmy-Test-Mode");

        Swimmy2023RobotSubsystem robotsys = (Swimmy2023RobotSubsystem) ctrl.getRobot().getRobotSubsystem();
        LimeLightSubsystem limelight = robotsys.getLimeLight();
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

        grabberGrabMotor = grabber.getGrabSubsystem() ;
        grabberSpinMotor = grabber.getSpinSubsystem() ;

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

            case 6:
                addSubActionPair(swerve, new SwerveAlignDriveBaseAction(swerve, limelight, 3.0), true);
                break;
                
            //
            // ARM test modes
            //
            case 10:
                addSubActionPair(armUpper, new MotorEncoderPowerAction(armUpper, getDouble("power"), getDouble("duration")), true) ;
                break ;

            case 11:
                addSubActionPair(armUpper, new MotorEncoderGotoAction(armUpper, getDouble("height"), true), true) ;
                addSubActionPair(armLower, new MotorEncoderPowerAction(armLower, getDouble("power"), getDouble("duration")), true) ;
                break ;                

            case 12:
                {
                    TrapezoidalProfileConfig cfg = new TrapezoidalProfileConfig(100000, -100000, 100000) ;
                    addSubActionPair(armUpper, new MotorEncoderGotoAction(armUpper, getDouble("target"), cfg, true), true) ;
                }
                break ;

            case 13:
                {
                    SCurveConfig cfg = new SCurveConfig(1000000, 200000, 100000);
                    addSubActionPair(armUpper, new MotorEncoderGotoAction(armUpper, getDouble("height"), cfg, true), true) ;
                    addSubActionPair(armLower, new MotorEncoderGotoAction(armLower, getDouble("target"), cfg, true), true) ;
                }
                break ;

            case 19:
                addSubActionPair(arm, new ArmStaggeredGotoAction(arm, "place:top:cone:extend", true), true) ;
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
                
            case 25:
                addSubActionPair(grabber, new GrabberStartCollectAction(grabber, GamePiece.Cone), true) ;
                break ;

            case 26:
                addSubActionPair(grabber, new GrabberStartCollectAction(grabber, GamePiece.Cone), true) ;
                addAction(new DelayAction(getAutoController().getRobot(), 2.0));
                addSubActionPair(grabber, new GrabberStopCollectAction(grabber), true) ;
                break ;

            case 27:
                break ;

            case 28:
                break ;

            case 30:
                addSubActionPair(gpm, new GPMCollectAction(gpm, RobotOperation.GamePiece.Cone, false), true);
                break ;

            case 31:
                addSubActionPair(gpm, new GPMCollectAction(gpm, RobotOperation.GamePiece.Cube, false), true);
                break ;     

            case 32:
                addSubActionPair(gpm, new GPMCollectAction(gpm, RobotOperation.GamePiece.Cone, true), true);
                break ;

            case 33:
                addSubActionPair(gpm, new GPMCollectAction(gpm, RobotOperation.GamePiece.Cube, true), true);
                break ;   

            case 78:
                addSubActionPair(swerve, new SwerveAlignRobotAction(swerve, limelight), true);
                break ;

            case 79:
                addSubActionPair(grabber, new GrabberGrabGampieceAction(grabber, GamePiece.Cube), true);
                addSubActionPair(arm, new ArmGotoAction(arm, 0, 20000), true);
                addAction(new DelayAction(gpm.getRobot(), getDouble("delay")));
                addSubActionPair(grabberGrabMotor, new MotorEncoderHoldAction(grabberGrabMotor, getDouble("position")), true);
                break ;

            case 80:
                {
                    addSubActionPair(grabber, new GrabberGrabGampieceAction(grabber, GamePiece.Cone), true);
                    addAction(new DelayAction(gpm.getRobot(), getDouble("delay")));
                    addSubActionPair(gpm, new GPMPlaceAction(gpm, Location.Top, GamePiece.Cone, true), true);
                }
                break;

            case 81:
                {
                    addSubActionPair(grabber, new GrabberGrabGampieceAction(grabber, GamePiece.Cube), true);
                    addAction(new DelayAction(gpm.getRobot(), getDouble("delay")));
                    addSubActionPair(gpm, new GPMPlaceAction(gpm, Location.Middle, GamePiece.Cube, true), true);
                }
                break;

            case 82:
                addSubActionPair(gpm, new GPMCollectAction(gpm, GamePiece.Cube, false), true) ;
                break;

            case 83:
                addSubActionPair(gpm, new GPMCollectAction(gpm, GamePiece.Cube, false), true) ;
                break;                
                
            case 84:
                addSubActionPair(gpm, new GPMCollectAction(gpm, GamePiece.Cone, true), true);
                break;  
                
            case 85:
                addSubActionPair(gpm, new GPMCollectAction(gpm, GamePiece.Cube, true), true);
                break;                    
        }
    }

    @Override
    public Pose2d getInitialPose() {
        return initial_pose_ ;
    }
    
}
