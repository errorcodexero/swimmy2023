package frc.robot.subsystems.toplevel;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.arm.ArmStaggeredGotoAction;
import frc.robot.subsystems.gpm.GPMCollectAction;

public class ManualCollectOpCtrl extends OperationCtrl {

    public final static boolean AddLiftStep = false ;

    private enum State {
        Idle,
        WaitingForButton,
        WaitingForCollect,
        DriveBack,
    }

    private State state_ ;

    private GPMCollectAction collect_action_ ;
    private ArmStaggeredGotoAction stow_arm_;
    private XeroTimer drive_back_timer_ ;
    
    public ManualCollectOpCtrl(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws Exception {
        super(sub, oper) ;

        state_ = State.Idle;

        collect_action_ = new GPMCollectAction(sub.getGPM(), oper.getGamePiece(), oper.getGround());

        stow_arm_ = new ArmStaggeredGotoAction(sub.getGPM().getArm(), "collect:retract-shelf", false);

        drive_back_timer_ = new XeroTimer(sub.getRobot(), "collect-back-timer", 0.5);
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

            case WaitingForButton:
                stateWaitingForButton() ;
                break ;

            case WaitingForCollect:
                stateWaitingForCollect() ;
                break ;

            case DriveBack:
                stateDriveBack() ;
                break ;
        }

        if (state_ != orig) {
            MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
            logger.add("ManualCollectOpCtrl State Changes: " + orig.toString() + " -> " + state_.toString());
            logger.endMessage();
        }
    }

    @Override
    public void abort() throws BadParameterTypeException, MissingParameterException {
        switch(state_) {
            case Idle:
                break ;

            case WaitingForButton:
                break ;

            case WaitingForCollect:
                break ;

            case DriveBack:
                getRobotSubsystem().getOI().enableGamepad() ;
                getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
                break ;
        }

        setDone();
        state_ = State.Idle;
    }

    private void stateIdle() {
        state_ = State.WaitingForButton;
    }

    private void stateWaitingForButton() throws BadParameterTypeException, MissingParameterException {
        if (getRobotSubsystem().getOI().isCollectPressed()) {
            getRobotSubsystem().getGPM().setAction(collect_action_);
            state_ = State.WaitingForCollect ;
        }
    }

    private void stateWaitingForCollect() throws BadParameterTypeException, MissingParameterException {
        if (collect_action_.isDone()) {
            ChassisSpeeds speed = new ChassisSpeeds(-3.0, 0.0, 0.0) ;
            getRobotSubsystem().getOI().disableGamepad();
            getRobotSubsystem().getSwerve().drive(speed) ;
            drive_back_timer_.start() ;
            state_ = State.DriveBack ;
        }
    }

    private void stateDriveBack() {
        if (drive_back_timer_.isExpired()) {
            getRobotSubsystem().getOI().enableGamepad();
            getRobotSubsystem().getOI().getGamePad().rumble(1.0, 0.5);
            getRobotSubsystem().getSwerve().drive(new ChassisSpeeds()) ;
            getRobotSubsystem().getGPM().getArm().setAction(stow_arm_);
            state_ = State.Idle ;
            setDone() ;
        }
    }
}
