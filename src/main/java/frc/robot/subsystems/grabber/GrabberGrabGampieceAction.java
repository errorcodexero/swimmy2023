package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class GrabberGrabGampieceAction extends Action {
    private GrabberSubsystem sub_ ;
    private double hold_power_ ;
    private XeroTimer timer_;
    private MotorEncoderPowerAction motor_action_ ;

    public GrabberGrabGampieceAction(GrabberSubsystem sub) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;
        hold_power_ = sub.getSettingsValue("close:hold-power").getDouble();

        double delay = sub.getSettingsValue("close:startup-delay").getDouble();
        timer_ = new XeroTimer(sub.getRobot(), "grabgamepiece", delay);

        motor_action_ = new MotorEncoderPowerAction(sub.getGrabSubsystem(), hold_power_);
    }

    @Override
    public void start() throws Exception {
        super.start();
        
        sub_.getGrabSubsystem().setAction(motor_action_, true);
        timer_.start() ;
    }

    @Override
    public void run() throws Exception { 
        super.run();

        if (timer_.isExpired()) {
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberGrabGampieceAction";
    }
}
