package frc.robot.subsystems.toplevel;

import org.xero1425.base.subsystems.swerve.common.SwerveDriveToPoseAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.gpm.GPMPlaceAction;

public class AutoPlaceOpCtrl extends OperationCtrl {

    private enum State {
        Idle,
        LookingForTag,
        DrivingToLocation,
        WaitingOnArm,
        DroppingPiece
    }

    private double april_tag_action_threshold_ ;
    private State state_ ;
    private Pose2d target_pose_ ;

    private SwerveDriveToPoseAction drive_to_action_ ;
    private GPMPlaceAction place_action_ ;
    
    public AutoPlaceOpCtrl(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws BadParameterTypeException, MissingParameterException {
        super(sub, oper) ;

        april_tag_action_threshold_ = sub.getSettingsValue("april-tag-action-threshold").getDouble() ;
        state_ = State.Idle ;

        place_action_ = new GPMPlaceAction(sub.getGPM(), oper.getLocation(), oper.getGamePiece(), false);
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
                place_action_.cancel();
                break;
        }

        setDone();
        state_ = State.Idle;        
    }       

    private void stateIdle() {
        state_ = State.LookingForTag ;
    }

    private Pose2d[] computeDrivePathEndPoints(Pose2d robotpos, Pose2d destpos) {
        Pose2d [] ret = new Pose2d[2];

        double dy = destpos.getY() - robotpos.getY() ;
        double dx = destpos.getX() - robotpos.getX() ;
        double angle = Math.atan2(dy, dx);

        ret[0] = new Pose2d(robotpos.getX(), robotpos.getY(), Rotation2d.fromRadians(angle));
        ret[1] = new Pose2d(destpos.getX(), destpos.getY(), Rotation2d.fromRadians(angle));

        return ret;
    }

    private void stateLookingForTag() throws BadParameterTypeException, MissingParameterException {
        int tag = getRobotSubsystem().getFieldData().getGridTag(getOper().getAprilTag());
        double dist = getRobotSubsystem().getLimeLight().distantToTag(tag) ;

        if (dist < april_tag_action_threshold_) {
            getRobotSubsystem().getOI().disableGamepad();
            // getRobotSubsystem().getOI().getGamePad().rumble(1.0, 2.0);

            getRobotSubsystem().getSwerve().enableVision(false);

            target_pose_ = getRobotSubsystem().getFieldData().getGridPose(getOper().getAprilTag(), getOper().getSlot());

            Pose2d [] pts = computeDrivePathEndPoints(getRobotSubsystem().getSwerve().getPose(), target_pose_);
            drive_to_action_ = new SwerveDriveToPoseAction(getRobotSubsystem().getSwerve(), pts);
            getRobotSubsystem().getSwerve().setAction(drive_to_action_);
            // getRobotSubsystem().getGPM().setAction(place_action_);
            state_ = State.DrivingToLocation ;
        }
    }

    private void stateDrivingToLocation() {
        if (drive_to_action_.isDone()) {
            getRobotSubsystem().getSwerve().enableVision(true);
            getRobotSubsystem().getGPM().setAction(place_action_);
            state_ = State.WaitingOnArm ;
        }
    }

    private void stateWaitingOnArm() {
        if (place_action_.isReadyToDrop()) {
            place_action_.dropGamePiece();
            state_ = State.DroppingPiece ;
        }
    }

    private void stateDroppingPiece() {
        if (place_action_.isDone()) {
            state_ = State.Idle ;
            getRobotSubsystem().getOI().enableGamepad();
            // getRobotSubsystem().getOI().getGamePad().rumble(1.0, 2.0);
            setDone();
        }
    }
}
