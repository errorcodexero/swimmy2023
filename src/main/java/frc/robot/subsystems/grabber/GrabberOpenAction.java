package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;

public class GrabberOpenAction extends Action {

    private GrabberSubsystem sub_ ;

    public GrabberOpenAction(GrabberSubsystem sub) {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;
    }

    @Override
    public void start() {
        sub_.open() ;
        setDone() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberOpenAction" ;
    }
}
