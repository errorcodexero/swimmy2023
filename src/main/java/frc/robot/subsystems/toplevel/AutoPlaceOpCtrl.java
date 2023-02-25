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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.gpm.GPMPlaceAction;
import frc.robot.subsystems.toplevel.RobotOperation.Action;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class AutoPlaceOpCtrl extends OperationCtrl {

    private enum State {
        Idle,
        LookingForTag,
        DrivingToLocation,
        WaitingOnVision,
        WaitingOnArm,
        DroppingPiece
    }

    private double april_tag_action_threshold_ ;
    private State state_ ;
    private Pose2d target_pose_ ;

    private SwerveDrivePathAction drive_to_action_ ;
    private GPMPlaceAction place_action_ ;
    private MotorEncoderPowerAction spit_cube_action_ ;

    private XeroTimer vision_timer_ ;
    
    public AutoPlaceOpCtrl(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws BadParameterTypeException, MissingParameterException {
        super(sub, oper) ;

        april_tag_action_threshold_ = sub.getSettingsValue("april-tag-action-threshold").getDouble() ;
        state_ = State.Idle ;

        vision_timer_ = new XeroTimer(sub.getRobot(), "vision/timer", 0.5);

        if (oper.getAction() == Action.Place && oper.getGamePiece() == GamePiece.Cube && oper.getLocation() == Location.Bottom) {
            double power = getRobotSubsystem().getSettingsValue("spit-cube:power").getDouble();
            double duration = getRobotSubsystem().getSettingsValue("spit-cube:duration").getDouble();
            spit_cube_action_ = new MotorEncoderPowerAction(getRobotSubsystem().getGPM().getGrabber().getSpinSubsystem(), power, duration);
        }
        else {
            place_action_ = new GPMPlaceAction(sub.getGPM(), oper.getLocation(), oper.getGamePiece(), false);
        }
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
                stateIdle() ;
                break ;

            case LookingForTag:
                stateLookingForTag() ;
                break ;

            case DrivingToLocation:
                stateDrivingToLocation() ;
                break;

            case WaitingOnVision:
                stateWaitingForVision() ;
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
                break ;

            case LookingForTag:
                break ;

            case WaitingOnVision:
                break ;

            case DrivingToLocation:
                getRobotSubsystem().getOI().enableGamepad() ;
                // getRobotSubsystem().getOI().getGamePad().rumble(1.0, 2.0);
                drive_to_action_.cancel() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                break ;

            case WaitingOnArm:
                break;

            case DroppingPiece:
                getRobotSubsystem().getOI().enableGamepad() ;
                // getRobotSubsystem().getOI().getGamePad().rumble(1.0, 2.0);
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

        Pose2d p2 ;
        Pose2d p0 = new Pose2d(robotpos.getX(), robotpos.getY(), Rotation2d.fromRadians(angle)) ;
        Pose2d p1 = new Pose2d(destpos.getX(), destpos.getY(), Rotation2d.fromRadians(angle)) ;

        if (DriverStation.getAlliance() == Alliance.Red) {
            p2 = new Pose2d(p1.getX() + 0.07, p1.getY(), target_pose_.getRotation());
        }
        else {
            p2 = new Pose2d(p1.getX() - 0.07, p1.getY(), target_pose_.getRotation());
        }

        ret.add(p0);
        ret.add(p1) ;
        ret.add(p2) ;

        return ret;
    }

    private void stateLookingForTag() throws BadParameterTypeException, MissingParameterException {
        int tag = getRobotSubsystem().getFieldData().getGridTag(getOper().getAprilTag());
        double dist = getRobotSubsystem().getLimeLight().distantToTag(tag) ;

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

        if (vision_timer_.isExpired()) {

            // getRobotSubsystem().getOI().getGamePad().rumble(1.0, 2.0);
            getRobotSubsystem().getSwerve().enableVision(false);
            target_pose_ = getRobotSubsystem().getFieldData().getGridPose(getOper().getAprilTag(), getOper().getSlot());

            List<Pose2d> pts = computeDrivePathPoints(getRobotSubsystem().getSwerve().getPose(), target_pose_);
            List<Translation2d> interior = new ArrayList<Translation2d>() ;
            interior.add(pts.get(1).getTranslation());
            drive_to_action_ = new SwerveDrivePathAction(getRobotSubsystem().getSwerve(), pts.get(0), interior, pts.get(2), target_pose_.getRotation());

            getRobotSubsystem().getSwerve().setAction(drive_to_action_);
            if (place_action_ != null) {
                getRobotSubsystem().getGPM().setAction(place_action_);
            }
            state_ = State.DrivingToLocation ;
        }
        else {
            MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger() ;
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
            getRobotSubsystem().getSwerve().enableVision(true);
            state_ = State.WaitingOnArm ;
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
