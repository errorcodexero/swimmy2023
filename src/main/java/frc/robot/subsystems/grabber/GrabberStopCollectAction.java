package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class GrabberStopCollectAction extends Action {

    private GrabberSubsystem sub_ ;

    public GrabberStopCollectAction(GrabberSubsystem sub) {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getGrabSubsystem().setAction(new MotorEncoderPowerAction(sub_.getGrabSubsystem(), 0.2), true);
        sub_.getSpinSubsystem().setAction(new MotorEncoderPowerAction(sub_.getSpinSubsystem(), 0), true);
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
        return spaces(indent) + "GrabberStopCollectAction" ;
    }
}