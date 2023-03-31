package frc.robot.automodes;

import java.util.function.Supplier;

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
import org.xero1425.misc.TrapezoidalProfileConfig;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.arm.ArmGotoAction;
import frc.robot.subsystems.arm.ArmStaggeredGotoAction;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.gpm.GPMCollectAction;
import frc.robot.subsystems.gpm.GPMPlaceAction;
import frc.robot.subsystems.gpm.GPMStowAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.grabber.GrabberGrabGampieceAction;
import frc.robot.subsystems.grabber.GrabberStartCollectAction;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveBalancePlatform;
import frc.robot.subsystems.swerve.SwerveDrivePathToGamePiece;
import frc.robot.subsystems.swerve.SwerveDriveBalancePlatform.XYDirection;
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

            case 7:
                addSubActionPair(swerve, new SwerveHolonomicPathFollower(swerve, getString("name"), true, 1.0), true) ;
                break ;

            case 8:
                {
                    String path = getString("name");
                    Supplier<Boolean> fun = () -> { return robotsys.getGPM().getGrabber().getSensor() ; } ;
                    addSubActionPair(swerve, new SwerveDrivePathToGamePiece(robotsys.getLimeLight(), 3, swerve, path, true, 0.1, fun), true);
                }
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
                    TrapezoidalProfileConfig cfg = new TrapezoidalProfileConfig(100000, -100000, 100000) ;
                    addSubActionPair(armUpper, new MotorEncoderGotoAction(armUpper, getDouble("height"), cfg, true), true) ;
                    addSubActionPair(armLower, new MotorEncoderGotoAction(armLower, getDouble("target"), cfg, true), true) ;
                }
                break ;
            
            case 17:
                addSubActionPair(arm.getUpperSubsystem(), new MotorEncoderGotoAction(arm.getUpperSubsystem(), 230000.0, true), true);
                addSubActionPair(arm.getLowerSubsystem(), new MotorEncoderGotoAction(arm.getLowerSubsystem(), 2000.0, true), true);
                addSubActionPair(arm.getUpperSubsystem(), new MotorEncoderGotoAction(arm.getUpperSubsystem(), 2000.0, true), true);
                break; 

            case 18:
                addSubActionPair(arm, new ArmStaggeredGotoAction(arm, getString("profile1"), false), true);
                addAction(new DelayAction(arm.getRobot(), getDouble("delay")));
                addSubActionPair(arm, new ArmStaggeredGotoAction(arm, getString("profile2"), false), true);
                break ;

            case 19:
                addSubActionPair(arm, new ArmStaggeredGotoAction(arm, "place:middle:cone:extend", false), true) ;
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
                
            case 75:
                addSubActionPair(arm, new ArmStaggeredGotoAction(arm, "place:top:cone:extend", false), true);
                addAction(new DelayAction(getAutoController().getRobot(), 2.0));
                addSubActionPair(arm, new ArmStaggeredGotoAction(arm, "place:top:cone:retract", false), true);
                break ;

            //
            // Test shooting cubes
            //
            // The position of the ARM is controlled by the 'shoot' profile under 'subsystems:arm' in the settings file.
            // The shooting is controlled by the value 'grabber-power' in the test mode data  in the settings file under test
            // 76.  The duration of time is in the same place and given as 'duration'.
            //
            // The cube will need to be fed in by hand when the test mode is started as we cannot hold a cube on the shelf
            // right now.
            //
            case 76:
                //
                // Feed in a cube by hand
                //
                addSubActionPair(grabber, new GrabberGrabGampieceAction(grabber, GamePiece.Cube, false), true);

                //
                // Let the mentor get their hand out of the way
                //
                addAction(new DelayAction(getAutoController().getRobot(), getDouble("grab-delay")));

                //
                // Position the ARM for shooting, uses the profile "subsytems:arm:shoot" to position the arm
                //
                addSubActionPair(arm, new ArmStaggeredGotoAction(arm, "shoot:top:cube:extend", false), true);

                //
                // Let the arm settings before shooting
                //
                //addAction(new DelayAction(getAutoController().getRobot(), getDouble("settle-delay")));

                //
                // Shoot the cube by running the grabbers backwards.  May need to experiment with opening them in parallel
                // to get better results
                //
                addSubActionPair(grabberSpinMotor, new MotorEncoderPowerAction(grabberSpinMotor, getDouble("grabber-power"), getDouble("duration")), true);

                //
                // Move the ARM back to the stowed position
                //
                //addSubActionPair(arm, new ArmStaggeredGotoAction(arm, "shoot:top:cube:retract", false), true);
                addSubActionPair(gpm, new GPMStowAction(gpm), true);
                break;

            //
            // Test balance action on charging station
            //
            case 77:
                addSubActionPair(swerve, new SwerveDriveBalancePlatform(swerve, XYDirection.XDirection), true) ;
                break;

            case 79:
                addSubActionPair(grabber, new GrabberGrabGampieceAction(grabber, GamePiece.Cube, false), true);
                addSubActionPair(arm, new ArmGotoAction(arm, 0, 20000), true);
                addAction(new DelayAction(gpm.getRobot(), getDouble("delay")));
                addSubActionPair(grabberGrabMotor, new MotorEncoderHoldAction(grabberGrabMotor, getDouble("position")), true);
                break ;

            case 80:
                {
                    addSubActionPair(grabber, new GrabberGrabGampieceAction(grabber, GamePiece.Cone, false), true);
                    addAction(new DelayAction(gpm.getRobot(), getDouble("delay")));
                    addSubActionPair(gpm, new GPMPlaceAction(gpm, Location.Middle, GamePiece.Cone, true), true);
                }
                break;

            case 81:
                {
                    addSubActionPair(grabber, new GrabberGrabGampieceAction(grabber, GamePiece.Cone, false), true);
                    addAction(new DelayAction(gpm.getRobot(), getDouble("delay")));
                    addSubActionPair(gpm, new GPMPlaceAction(gpm, Location.Top, GamePiece.Cone, true), true);
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
