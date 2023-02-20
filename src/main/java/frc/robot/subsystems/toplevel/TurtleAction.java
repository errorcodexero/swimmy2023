package frc.robot.subsystems.toplevel;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.gpm.GPMStowAction;

public class TurtleAction extends Action {
    private Swimmy2023RobotSubsystem sub_ ;
    private GPMStowAction stow_action_ ;

    public TurtleAction(Swimmy2023RobotSubsystem sub) throws MissingParameterException, BadParameterTypeException {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;
        stow_action_ = new GPMStowAction(sub_.getGPM());
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getGPM().setAction(stow_action_, true);
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (stow_action_.isDone()) {
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "TurtleAction" ;
    }
}
