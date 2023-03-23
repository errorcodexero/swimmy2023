package frc.robot.subsystems.toplevel;

import java.util.ArrayList;
import java.util.List;

import org.xero1425.base.misc.XeroElapsedTimer;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.swerve.common.SwerveDrivePathAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.gpm.GPMPlaceAction;
import frc.robot.subsystems.swerve.SwerveLinearAlignAction;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class AutoPlaceOpCtrl extends OperationCtrl {

    private final boolean AddAlignStep = false ;
    private final boolean AddDriveForward = true ;
    private final boolean AddSettlingDelay = true ;

    private enum State {
        Idle,
        LookingForTag,
        WaitingOnVision,
        DrivingToLocation,
        AlignRobot,
        AlignWheels,
        DriveForward,
        WaitingOnArm,
        SettlingDelay,
        DroppingPiece,
    }

    private double april_tag_action_threshold_ ;
    private double april_tag_arm_action_threshold_ ;
    private State state_ ;
    private Pose2d target_pose_ ;
    private boolean do_drive_forward_ ;
    private boolean do_settling_delay_ ;

    private SwerveDrivePathAction drive_to_action_ ;
    private GPMPlaceAction place_action_ ;
    private SwerveLinearAlignAction align_action_ ;

    private XeroTimer vision_timer_ ;
    private XeroTimer forward_timer_ ;
    private XeroTimer settling_timer_ ;
    private XeroTimer wheels_timer_ ;

    private XeroElapsedTimer overall_timer_ ;   // Measure time since auto takes over
    
    public AutoPlaceOpCtrl(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws BadParameterTypeException, MissingParameterException {
        super(sub, oper) ;

        april_tag_action_threshold_ = sub.getSettingsValue("april-tag-place-action-threshold").getDouble() ;
        april_tag_arm_action_threshold_ = sub.getSettingsValue("april-tag-place-arm-action-threshold").getDouble() ;
        state_ = State.Idle ;

        if (getRobotSubsystem().getRobot().isAutonomous())
            vision_timer_ = new XeroTimer(sub.getRobot(), "vision/timer", 0.3);
        else
            vision_timer_ = new XeroTimer(sub.getRobot(), "vision/timer", 0.2);

        settling_timer_ = new XeroTimer(sub.getRobot(), "settling", 0.3) ;
        align_action_ = new SwerveLinearAlignAction(getRobotSubsystem().getSwerve(), getRobotSubsystem().getLimeLight()) ;

        place_action_ = new GPMPlaceAction(sub.getGPM(), oper.getLocation(), oper.getGamePiece(), false);

        forward_timer_ = new XeroTimer(sub.getRobot(), "forward", 0.3) ;
        wheels_timer_ = new XeroTimer(sub.getRobot(), "wheels", 0.1) ;

        overall_timer_ = new XeroElapsedTimer(sub.getRobot()) ;
    }

    @Override
    public void start() throws BadParameterTypeException, MissingParameterException {
        super.start();
        do_drive_forward_ = AddDriveForward ;
        do_settling_delay_ = AddSettlingDelay ;
        if (getOper().getGamePiece() == GamePiece.Cube) {
            if (getOper().getLocation() == Location.Middle) {
                //do_drive_forward_ = false ;
            }
            do_settling_delay_ = false ;
        }
        state_ = State.Idle ;
    }

    @Override
    public void run() throws BadParameterTypeException, MissingParameterException {
        State orig = state_ ;

        switch(state_) {
            case Idle:
                stateIdle() ;
                break ;

            case LookingForTag:
                stateLookingForTag() ;
                break ;

            case WaitingOnVision:
                stateWaitingForVision() ;
                break ;                

            case DrivingToLocation:
                stateDrivingToLocation() ;
                break;

            case AlignRobot:
                stateAlignRobot() ;
                break ;

            case AlignWheels:
                stateAlignWheels() ;
                break ;

            case DriveForward:
                stateDriveForward() ;
                break ;

            case WaitingOnArm:
                stateWaitingOnArm() ;
                break;

            case SettlingDelay:
                stateSettlingDelay() ;
                break ;                

            case DroppingPiece:
                stateDroppingPiece() ;
                break;
        }

        if (state_ != orig) {
            MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
            logger.add("AutoPlaceOpCtrl State Changes: " + orig.toString() + " -> " + state_.toString());
            logger.endMessage();
        }
    }

    @Override
    public void abort() throws BadParameterTypeException, MissingParameterException {
        switch(state_) {
            case Idle:
                break ;

            case LookingForTag:
                break ;

            case WaitingOnVision:
                break ;

            case AlignWheels:
                break ;

            case AlignRobot:
                getRobotSubsystem().getOI().enableGamepad() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                drive_to_action_.cancel() ;
                getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
                getRobotSubsystem().getLimeLight().setPipeline(0);
                break;

            case DrivingToLocation:
                getRobotSubsystem().getOI().enableGamepad() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                drive_to_action_.cancel() ;
                getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
                break ;                

            case DriveForward:
                drive_to_action_.cancel() ;
                getRobotSubsystem().getOI().enableGamepad() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
                break ;

            case WaitingOnArm:
                getRobotSubsystem().getOI().enableGamepad() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                break;

            case SettlingDelay:                
                getRobotSubsystem().getOI().enableGamepad() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                break;

            case DroppingPiece:
                getRobotSubsystem().getOI().enableGamepad() ;
                place_action_.cancel();
                break;
        }

        setDone();
        state_ = State.Idle;        
    }       

    private void stateIdle() {
        state_ = State.LookingForTag ;
    }

    private List<Pose2d> computeDrivePathPoints(Pose2d robotpos, Pose2d destpos) {
        List<Pose2d> ret = new ArrayList<Pose2d>();

        double dy = destpos.getY() - robotpos.getY() ;
        double dx = destpos.getX() - robotpos.getX() ;
        double angle = Math.atan2(dy, dx);

        Pose2d p0 = new Pose2d(robotpos.getX(), robotpos.getY(), Rotation2d.fromRadians(angle)) ;
        Pose2d p1 = new Pose2d(destpos.getX(), destpos.getY(), Rotation2d.fromRadians(angle)) ;

        ret.add(p0);
        ret.add(p1) ;

        return ret;
    }

    private void stateLookingForTag() throws BadParameterTypeException, MissingParameterException {
        int tag = getRobotSubsystem().getFieldData().getGridTag(Alliance.Invalid, getOper().getAprilTag());
        double dist = getRobotSubsystem().getLimeLight().distantToTag(tag) ;

        MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
        logger.add("Waiting For Tag");
        logger.add("tag", tag) ;
        logger.add("dist", dist) ;
        logger.endMessage();

        if (dist < april_tag_action_threshold_) {
            getRobotSubsystem().getSwerve().cancelAction();
            getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
            getRobotSubsystem().getOI().disableGamepad();
            getRobotSubsystem().getOI().getGamePad().rumble(1.0, 0.5);

            getRobotSubsystem().getGPM().setAction(place_action_, true);

            overall_timer_.reset() ;
            state_ = State.WaitingOnVision ;
            vision_timer_.start() ;
        }
        else if (dist < april_tag_arm_action_threshold_ && getOper().getGamePiece() == GamePiece.Cube) {
            if (getRobotSubsystem().getGPM().getAction() == null) {
                getRobotSubsystem().getGPM().setAction(place_action_, true);
            }
        }
    }

    private void stateWaitingForVision() throws BadParameterTypeException, MissingParameterException {
        int tag = getRobotSubsystem().getFieldData().getGridTag(Alliance.Invalid, getOper().getAprilTag());
        double dist = getRobotSubsystem().getLimeLight().distantToTag(tag) ;

        MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger();
        if (vision_timer_.isExpired()) {

            getRobotSubsystem().getSwerve().enableVision(false);
            target_pose_ = getRobotSubsystem().getFieldData().getGridPose(Alliance.Invalid, getOper().getAprilTag(), getOper().getSlot());

            double max_v;
            double max_a;
            if (getOper().getGamePiece() == GamePiece.Cone) {
                max_v = 1.0;
                max_a = 1.0;
            } else {
                max_v = 2.0;
                max_a = 2.0;
            }

            List<Pose2d> pts = computeDrivePathPoints(getRobotSubsystem().getSwerve().getPose(), target_pose_);
            List<Translation2d> interior = new ArrayList<Translation2d>() ;
            drive_to_action_ = new SwerveDrivePathAction(getRobotSubsystem().getSwerve(), pts.get(0), interior, pts.get(1), target_pose_.getRotation(), max_a, max_v);

            getRobotSubsystem().getSwerve().setAction(drive_to_action_, true);


            if (AddAlignStep) {
                getRobotSubsystem().getLimeLight().setPipeline(1);
            }
            state_ = State.DrivingToLocation ;
        }
        else {
            logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
            logger.add("Waiting On Vision") ;
            logger.add("tag", tag) ;
            logger.add("dist", dist) ;
            logger.add("vision", getRobotSubsystem().getLimeLight().getBlueBotPose().toPose2d());
            logger.add("db", getRobotSubsystem().getSwerve().getPose());
            logger.endMessage();
        }
    }

    private void stateDrivingToLocation() {
        if (drive_to_action_.isDone()) {
            if (AddAlignStep) {
                getRobotSubsystem().getSwerve().setAction(align_action_, true) ;
                state_ = State.AlignRobot ;
            } else {
                double [] angles = new double[] { 0.0, 0.0, 0.0, 0.0} ;
                double [] power = new double[] { 0.0, 0.0, 0.0, 0.0} ;
    
                getRobotSubsystem().getLimeLight().setPipeline(0);
    
                getRobotSubsystem().getSwerve().setRawTargets(true, angles, power) ;
                state_ = State.AlignWheels ;
                wheels_timer_.start() ;
            }
        }
    }

    private void stateAlignRobot() {
        if (align_action_.isDone()) {
            double [] angles = new double[] { 0.0, 0.0, 0.0, 0.0} ;
            double [] power = new double[] { 0.0, 0.0, 0.0, 0.0} ;

            getRobotSubsystem().getLimeLight().setPipeline(0);

            getRobotSubsystem().getSwerve().setRawTargets(true, angles, power) ;
            state_ = State.AlignWheels ;
            wheels_timer_.start() ;
        }
    }

    private void stateAlignWheels() {
        if (!AddAlignStep || wheels_timer_.isExpired()) {
            if (do_drive_forward_) {
                ChassisSpeeds speed ;            
                double xspeed = 1.0;
                speed = new ChassisSpeeds(xspeed, 0.0, 0.0) ;
                getRobotSubsystem().getSwerve().drive(speed) ;
                forward_timer_.start() ;
                state_ = State.DriveForward;
            }
            else {
                if (do_settling_delay_) {
                    getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
                    getRobotSubsystem().getSwerve().enableVision(true);
                    settling_timer_.start() ;
                    state_ = State.SettlingDelay ;
                }
                else {
                    state_ = State.WaitingOnArm;                    
                }
            }
        }
    }

    private void stateDriveForward() {
        if (forward_timer_.isExpired()) {
            if (AddSettlingDelay) {
                getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
                getRobotSubsystem().getSwerve().enableVision(true);
                settling_timer_.start() ;
                state_ = State.SettlingDelay ;
            }
            else {
                state_ = State.WaitingOnArm;
            }
        }
    }

    private void stateSettlingDelay() {
        if (settling_timer_.isExpired()) {
            state_ = State.WaitingOnArm;
        }
    }

    private void stateWaitingOnArm() {
        MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
        logger.add("Waiting On Arm") ;
        logger.endMessage();

        if (place_action_.isReadyToDrop()) {

            logger.startMessage(MessageType.Info) ;
            logger.add("Dropping game piece") ;
            logger.add("uppper arm", getRobotSubsystem().getGPM().getArm().getUpperSubsystem().getPosition());
            logger.add("lower arm", getRobotSubsystem().getGPM().getArm().getLowerSubsystem().getPosition());
            logger.endMessage();
            place_action_.dropGamePiece();
            state_ = State.DroppingPiece ;
        }
    }

    private void stateDroppingPiece() {
        if (place_action_.isDone()) {
            state_ = State.Idle ;

            if (place_action_.isDropComplete()) {
                getRobotSubsystem().getOI().enableGamepad();
                getRobotSubsystem().getOI().getGamePad().rumble(1.0, 0.5);
            }

            MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
            logger.add("AutoPlaceOpCtrl duration: " + overall_timer_.elapsed());
            logger.endMessage();

            setDone();
        }
    }
}
