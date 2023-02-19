package frc.robot.subsystems.toplevel;

import frc.robot.subsystems.gpm.GPMCollectAction;

public class AutoCollectGroundOpCtrl extends OperationCtrl {

    private GPMCollectAction collect_action_ ;

    public AutoCollectGroundOpCtrl(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws Exception {
        super(sub, oper);

        collect_action_ = new GPMCollectAction(sub.getGPM(), true);
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
    }
}
