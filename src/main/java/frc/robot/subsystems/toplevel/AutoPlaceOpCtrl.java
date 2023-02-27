package frc.robot.subsystems.toplevel;

import java.util.ArrayList;
import java.util.List;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.swerve.common.SwerveDrivePathAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.gpm.GPMPlaceAction;
import frc.robot.subsystems.swerve.SwerveAlignRobotAction;
import frc.robot.subsystems.toplevel.RobotOperation.Action;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class AutoPlaceOpCtrl extends OperationCtrl {

    private enum State {
        Idle,
        LookingForTag,
        WaitingOnVision,
        DrivingToLocation,
        AlignRobot,
        DriveForward,
        SettlingDelay,
        WaitingOnArm,
        DroppingPiece,

        InfiniteLoop
    }

    private double april_tag_action_threshold_ ;
    private State state_ ;
    private Pose2d target_pose_ ;

    private SwerveDrivePathAction drive_to_action_ ;
    private GPMPlaceAction place_action_ ;
    private MotorEncoderPowerAction spit_cube_action_ ;
    private SwerveAlignRobotAction align_action_ ;

    private XeroTimer vision_timer_ ;
    private XeroTimer forward_timer_ ;
    private XeroTimer settling_timer_ ;
    
    public AutoPlaceOpCtrl(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws BadParameterTypeException, MissingParameterException {
        super(sub, oper) ;

        april_tag_action_threshold_ = sub.getSettingsValue("april-tag-action-threshold").getDouble() ;
        state_ = State.Idle ;

        vision_timer_ = new XeroTimer(sub.getRobot(), "vision/timer", 0.5);
        settling_timer_ = new XeroTimer(sub.getRobot(), "settling", 0.1) ;

        if (oper.getAction() == Action.Place && oper.getGamePiece() == GamePiece.Cube && oper.getLocation() == Location.Bottom) {
            double power = getRobotSubsystem().getSettingsValue("spit-cube:power").getDouble();
            double duration = getRobotSubsystem().getSettingsValue("spit-cube:duration").getDouble();
            spit_cube_action_ = new MotorEncoderPowerAction(getRobotSubsystem().getGPM().getGrabber().getSpinSubsystem(), power, duration);
        }
        else {
            place_action_ = new GPMPlaceAction(sub.getGPM(), oper.getLocation(), oper.getGamePiece(), false);
        }

        align_action_ = new SwerveAlignRobotAction(sub.getSwerve(), sub.getLimeLight());
        forward_timer_ = new XeroTimer(sub.getRobot(), "forward", 0.5) ;
    }

    @Override
    public void start() throws BadParameterTypeException, MissingParameterException {
        super.start();
        state_ = State.Idle ;
    }

    @Override
    public void run() throws BadParameterTypeException, MissingParameterException {
        State orig = state_ ;

        switch(state_) {
            case Idle:
            case InfiniteLoop:
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

            case DriveForward:
                stateDriveForward() ;
                break ;

            case SettlingDelay:
                stateSettlingDelay() ;
                break ;

            case WaitingOnArm:
                stateWaitingOnArm() ;
                break;

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
            case InfiniteLoop:
                break ;

            case LookingForTag:
                break ;

            case WaitingOnVision:
            case SettlingDelay:
                break ;

            case DriveForward:
                getRobotSubsystem().getOI().enableGamepad() ;
                drive_to_action_.cancel() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
                break ;

            case AlignRobot:
                break ;

            case DrivingToLocation:
                getRobotSubsystem().getOI().enableGamepad() ;
                drive_to_action_.cancel() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                break ;

            case WaitingOnArm:
                break;

            case DroppingPiece:
                getRobotSubsystem().getOI().enableGamepad() ;
                if (place_action_ != null) {
                    place_action_.cancel();
                }
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
        int tag = getRobotSubsystem().getFieldData().getGridTag(getOper().getAprilTag());
        double dist = getRobotSubsystem().getLimeLight().distantToTag(tag) ;

        MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
        logger.add("Waiting For Tag");
        logger.add("tag", tag) ;
        logger.add("dist", dist) ;
        logger.endMessage();

        if (dist < april_tag_action_threshold_) {
            getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
            getRobotSubsystem().getOI().disableGamepad();
            state_ = State.WaitingOnVision ;
            vision_timer_.start() ;
        }
    }

    private void stateWaitingForVision() throws BadParameterTypeException, MissingParameterException {
        int tag = getRobotSubsystem().getFieldData().getGridTag(getOper().getAprilTag());
        double dist = getRobotSubsystem().getLimeLight().distantToTag(tag) ;

        MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger();
        if (vision_timer_.isExpired()) {

            // getRobotSubsystem().getOI().getGamePad().rumble(1.0, 2.0);
            getRobotSubsystem().getSwerve().enableVision(false);
            target_pose_ = getRobotSubsystem().getFieldData().getGridPose(getOper().getAprilTag(), getOper().getSlot());

            List<Pose2d> pts = computeDrivePathPoints(getRobotSubsystem().getSwerve().getPose(), target_pose_);
            List<Translation2d> interior = new ArrayList<Translation2d>() ;
            drive_to_action_ = new SwerveDrivePathAction(getRobotSubsystem().getSwerve(), pts.get(0), interior, pts.get(1), target_pose_.getRotation());

            getRobotSubsystem().getSwerve().setAction(drive_to_action_);
            if (place_action_ != null) {
                getRobotSubsystem().getGPM().setAction(place_action_);
            }
            // getRobotSubsystem().getLimeLight().setPipeline(1);
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
            // getRobotSubsystem().getLimeLight().setPipeline(0);

            ChassisSpeeds speed ;            
            double xspeed = 0.5 ;
            speed = new ChassisSpeeds(xspeed, 0.0, 0.0) ;

            getRobotSubsystem().getSwerve().drive(speed) ;

            forward_timer_.start() ;
            state_ = State.DriveForward;
        }
    }

    private void stateAlignRobot() {
        if (align_action_.isDone()) {
            // getRobotSubsystem().getLimeLight().setPipeline(0);

            // ChassisSpeeds speed ;            
            // double xspeed = 0.25 ;
            // speed = new ChassisSpeeds(xspeed, 0.0, 0.0) ;

            // getRobotSubsystem().getSwerve().drive(speed) ;

            // forward_timer_.start() ;
            // state_ = State.DriveForward;

            state_ = State.InfiniteLoop ;
        }
    }

    private void stateDriveForward() {
        if (forward_timer_.isExpired()) {
            getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
            getRobotSubsystem().getSwerve().enableVision(true);
            settling_timer_.start() ;
            state_ = State.SettlingDelay ;
        }
    }

    private void stateSettlingDelay() {
        if (settling_timer_.isExpired()) {
            state_ = State.WaitingOnArm;
        }
    }

    private void stateWaitingOnArm() {
        if (place_action_ != null && place_action_.isReadyToDrop()) {
            place_action_.dropGamePiece();
            state_ = State.DroppingPiece ;
        }
        else if (spit_cube_action_ != null) {
            getRobotSubsystem().getGPM().getGrabber().getSpinSubsystem().setAction(spit_cube_action_);
        }
    }

    private void stateDroppingPiece() {
        if (place_action_ != null && place_action_.isDone()) {
            state_ = State.Idle ;
            getRobotSubsystem().getOI().enableGamepad();
            // getRobotSubsystem().getOI().getGamePad().rumble(1.0, 2.0);
            setDone();
        }
        else if (spit_cube_action_ != null && spit_cube_action_.isDone()) {
            state_ = State.Idle ;
            getRobotSubsystem().getOI().enableGamepad();
            // getRobotSubsystem().getOI().getGamePad().rumble(1.0, 2.0);
            setDone();
        }
    }
}
