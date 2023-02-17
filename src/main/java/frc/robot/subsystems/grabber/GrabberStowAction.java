package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderHoldAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class GrabberStowAction extends Action {

    private GrabberSubsystem sub_ ;
    private MotorEncoderHoldAction hold_action_ ;
    private MotorEncoderPowerAction spin_action_ ;

    public GrabberStowAction(GrabberSubsystem sub) throws MissingParameterException, BadParameterTypeException {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;

        hold_action_ = new MotorEncoderHoldAction(sub_.getGrabSubsystem(), 0) ;
        spin_action_ = new MotorEncoderPowerAction(sub_.getSpinSubsystem(), 0) ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getGrabSubsystem().setAction(hold_action_, true);
        sub_.getSpinSubsystem().setAction(spin_action_, true);
        setDone() ;
    }

    @Override
    public void run() throws Exception {
        super.run();
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberStowAction" ;
    }
    
}
