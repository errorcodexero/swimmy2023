package frc.robot.subsystems.toplevel;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.gpm.GPMCollectAction;

public class ManualCollectOpCtrl extends OperationCtrl {
    private enum State {
        Idle,
        WaitingOnDeploy,
        WaitingOnCollect
    };

    private State state_ ;
    private GPMCollectAction collect_action_ ;

    public ManualCollectOpCtrl(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws Exception {
        super(sub, oper) ;

        collect_action_ = new GPMCollectAction(sub.getGPM());
    }

    @Override
    public void start() throws BadParameterTypeException, MissingParameterException {
        super.start() ;

        state_ = State.WaitingOnDeploy;
    }

    @Override
    public void run() throws BadParameterTypeException, MissingParameterException {
        switch(state_) {
            case Idle:
                break;

            case WaitingOnDeploy:
                if (getRobotSubsystem().getOI().isActionButtonPressed()) {
                    getRobotSubsystem().getGPM().setAction(collect_action_);
                    state_ = State.WaitingOnCollect;
                }
                break;

            case WaitingOnCollect:
                if (collect_action_.isDone()) {
                    state_ = State.Idle;
                    setDone() ;
                }
                break;
        }
    }    

    @Override
    public void abort() throws BadParameterTypeException, MissingParameterException {
        switch(state_) {
            case Idle:
                break;

            case WaitingOnDeploy:
                break;

            case WaitingOnCollect:
                collect_action_.cancel() ;
                break;
        }

        state_ = State.Idle ;
        setDone() ;
    }
}
