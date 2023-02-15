package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderGotoAction;

public class GrabberOpenAction extends Action {

    private GrabberSubsystem sub_ ;

    public GrabberOpenAction(GrabberSubsystem sub) {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getGrabSubsystem().setAction(new MotorEncoderGotoAction(sub_.getGrabSubsystem(), "open", true), true);
    }

    @Override
    public void run() throws Exception {
        super.run();
        if (sub_.getGrabSubsystem().getAction().isDone()) {
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberOpenAction" ;
    }
}
