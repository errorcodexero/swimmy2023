package frc.robot.subsystems.toplevel;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.gpm.GPMPlaceAction;

public class ManualPlaceOpCtrl extends OperationCtrl {
    private enum State {
        Idle,
        WaitingOnDeploy,
        WaitingOnDrop,
        DroppingGamePiece,
    };

    private State state_ ;
    private GPMPlaceAction place_action_ ;

    public ManualPlaceOpCtrl(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws MissingParameterException, BadParameterTypeException {
        super(sub, oper) ;

        place_action_ = new GPMPlaceAction(sub.getGPM(), oper.getLocation(), oper.getGamePiece(), false);
    }

    @Override
    public void start() throws BadParameterTypeException, MissingParameterException {
        super.start();

        state_ = State.WaitingOnDeploy;
    }    

    @Override
    public void run() throws BadParameterTypeException, MissingParameterException {
        switch(state_) {
            case Idle:
                break;

            case WaitingOnDeploy:
                if (getRobotSubsystem().getOI().isActionButtonPressed()) {
                    getRobotSubsystem().getGPM().setAction(place_action_);
                    state_ = State.WaitingOnDrop;
                }
                break;

            case WaitingOnDrop:
                if (getRobotSubsystem().getOI().isDropButtonPressed() && place_action_.isReadyToDrop()) {
                    place_action_.dropGamePiece();
                    state_ = State.DroppingGamePiece;
                }
                break;

            case DroppingGamePiece:
                if (place_action_.isDone()) {
                    state_ = State.Idle ;
                    setDone();
                }
        }
    }

    @Override
    public void abort() throws BadParameterTypeException, MissingParameterException {
    }
}
