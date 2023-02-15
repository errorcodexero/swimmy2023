package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderHoldAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;

public class GrabberStartCollectAction extends Action {

    private GrabberSubsystem sub_ ;

    public GrabberStartCollectAction(GrabberSubsystem sub) {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getGrabSubsystem().setAction(new MotorEncoderHoldAction(sub_.getGrabSubsystem(), "positions:open"), true);
        sub_.getSpinSubsystem().setAction(new MotorEncoderPowerAction(sub_.getSpinSubsystem(), "power:spin"), true);
        setDone() ;
    }

    @Override
    public void run() throws Exception {
        super.run();
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberStartCollectAction" ;
    }
}