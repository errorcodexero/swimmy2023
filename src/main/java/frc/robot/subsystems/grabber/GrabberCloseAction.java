package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;

public class GrabberCloseAction extends Action {

    private GrabberSubsystem sub_ ;

    public GrabberCloseAction(GrabberSubsystem sub) {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;
    }

    @Override
    public void start() {
        sub_.close() ;
        setDone() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberOpenAction" ;
    }
}


