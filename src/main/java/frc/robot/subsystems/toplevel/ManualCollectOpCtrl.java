package frc.robot.subsystems.toplevel;

import org.xero1425.base.misc.XeroTimer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.arm.ArmStaggeredGotoAction;
import frc.robot.subsystems.gpm.GPMCollectAction;

public class ManualCollectOpCtrl extends OperationCtrl {

    private enum State {
        Idle,
        WaitForCollectButton,
        WaitingForGamePiece,
        DrivingBack
    }

    private GPMCollectAction collect_action_ ;
    private ArmStaggeredGotoAction stow_arm_ ;
    private XeroTimer drive_back_timer_ ;
    private State state_ ;
    
    public ManualCollectOpCtrl(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws Exception {
        super(sub, oper);

        state_ = State.Idle;
        collect_action_ = new GPMCollectAction(sub.getGPM(), oper.getGamePiece(), false);
        stow_arm_ = new ArmStaggeredGotoAction(sub.getGPM().getArm(), "collect:retract-shelf", false);
        drive_back_timer_ = new XeroTimer(sub.getRobot(), "collect-back-timer", 0.3);
    }

    @Override
    public void start() {
        state_ = State.WaitForCollectButton;
    }

    @Override
    public void run() {
        if (state_ == State.WaitForCollectButton) {
            getRobotSubsystem().getGPM().setAction(collect_action_);
            state_ = State.WaitingForGamePiece ;
        }
        else if (state_ == State.WaitingForGamePiece) {
            if (collect_action_.isDone()) {
                getRobotSubsystem().getOI().disableGamepad();
                ChassisSpeeds speed = new ChassisSpeeds(-3.0, 0.0, 0.0) ;
                getRobotSubsystem().getSwerve().drive(speed) ;
                drive_back_timer_.start() ;
                state_ = State.DrivingBack ;
            }
        }
        else if (state_ == State.DrivingBack) {
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

    @Override
    public void abort() {
        switch(state_) {
            case Idle:
            case WaitForCollectButton:
                break;

            case WaitingForGamePiece:
                getRobotSubsystem().getGPM().getGrabber().getSpinSubsystem().setPower(0.0);
                break;

            case DrivingBack:
                break;
        }
    }
}
