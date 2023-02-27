package frc.robot.subsystems.toplevel;

import frc.robot.subsystems.arm.ArmStaggeredGotoAction;
import frc.robot.subsystems.gpm.GPMCollectAction;
import frc.robot.subsystems.grabber.GrabberStowAction;

public class AutoCollectGroundOpCtrl extends OperationCtrl {

    private GPMCollectAction collect_action_ ;
    private ArmStaggeredGotoAction abort_arm_action_ ;
    private GrabberStowAction abort_grabber_action_ ;

    public AutoCollectGroundOpCtrl(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws Exception {
        super(sub, oper);

        collect_action_ = new GPMCollectAction(sub.getGPM(), RobotOperation.GamePiece.Cone, true);
        abort_arm_action_ = new ArmStaggeredGotoAction(sub.getGPM().getArm(), "collect:retract-ground", false) ;
        abort_grabber_action_ = new GrabberStowAction(sub.getGPM().getGrabber()) ;
    }

    @Override
    public void start() {
        getRobotSubsystem().getGPM().setAction(collect_action_);
    }

    @Override
    public void run() {
        if (collect_action_.isDone()) {
            setDone();
        }
    }

    @Override
    public void abort() {
        getRobotSubsystem().getGPM().cancelAction();
        getRobotSubsystem().getGPM().getArm().setAction(abort_arm_action_) ;
        getRobotSubsystem().getGPM().getGrabber().setAction(abort_grabber_action_) ;
    }
}
