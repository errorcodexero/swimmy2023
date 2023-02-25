package frc.robot.subsystems.toplevel;

import org.xero1425.base.subsystems.swerve.common.SwerveDriveToPoseAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.gpm.GPMCollectAction;

public class AutoCollectOpCtrl extends OperationCtrl {

    private enum State {
        Idle,
        LookingForTag,
        DrivingToLocation
    }

    private double april_tag_action_threshold_ ;
    private State state_ ;

    private GPMCollectAction collect_action_ ;
    private SwerveDriveToPoseAction drive_to_action_ ;


    private Pose2d target_pose_ ;
    
    public AutoCollectOpCtrl(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws Exception {
        super(sub, oper) ;

        april_tag_action_threshold_ = sub.getSettingsValue("april-tag-action-threshold").getDouble() ;
        state_ = State.Idle;

        collect_action_ = new GPMCollectAction(sub.getGPM(), oper.getGamePiece(), oper.getGround());
    }

    @Override
    public void start() throws BadParameterTypeException, MissingParameterException {
        super.start() ;
        
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
        }

        if (state_ != orig) {
            MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
            logger.add("AutoCollectOpCtrl State Changes: " + orig.toString() + " -> " + state_.toString());
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
        }

        setDone();
        state_ = State.Idle;
    }

    private void stateIdle() {
        state_ = State.LookingForTag ;
    }

    private void stateLookingForTag() throws BadParameterTypeException, MissingParameterException {
        int tag = getRobotSubsystem().getFieldData().getLoadingStationTag();

        if (getRobotSubsystem().getLimeLight().distantToTag(tag) < april_tag_action_threshold_) {
            getRobotSubsystem().getOI().disableGamepad();
            // getRobotSubsystem().getOI().getGamePad().rumble(1.0, 2.0);

            getRobotSubsystem().getSwerve().enableVision(false);

            target_pose_ = getRobotSubsystem().getFieldData().getLoadingStationPose(getOper().getSlot());

            drive_to_action_ = new SwerveDriveToPoseAction(getRobotSubsystem().getSwerve(), target_pose_);
            getRobotSubsystem().getSwerve().setAction(drive_to_action_);
            getRobotSubsystem().getGPM().setAction(collect_action_);
            state_ = State.DrivingToLocation ;
        }
    }

    private void stateDrivingToLocation() {
        if (collect_action_.isDone()) {
            getRobotSubsystem().getSwerve().enableVision(true);
            state_ = State.Idle ;
            getRobotSubsystem().getOI().enableGamepad();
            // getRobotSubsystem().getOI().getGamePad().rumble(1.0, 2.0);
            setDone() ;
        }
    }
}
