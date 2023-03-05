package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderHoldAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class GrabberGrabLoadedGamepieceAction extends Action {
    private GrabberSubsystem sub_ ;
    private MotorEncoderHoldAction hold_ ;
    private MotorEncoderPowerAction power_action_ ;
    private MotorEncoderPowerAction stop_spinner_action_;
    private XeroTimer timer_ ;

    public GrabberGrabLoadedGamepieceAction(GrabberSubsystem sub) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;

        double v = sub.getSettingsValue("start:position").getDouble() ;
        hold_ = new MotorEncoderHoldAction(sub_.getGrabSubsystem(), v) ;

        v = sub.getSettingsValue("start:delay").getDouble();
        timer_ = new XeroTimer(sub_.getRobot(), "gametimer", v) ;

        stop_spinner_action_ = new MotorEncoderPowerAction(sub.getSpinSubsystem(), 0.1);
        power_action_ = new MotorEncoderPowerAction(sub.getGrabSubsystem(), sub.getSettingsValue("close:hold-power").getDouble());
    }

    @Override
    public void start() throws Exception {
        sub_.getGrabSubsystem().setAction(hold_, true) ;
        timer_.start();
    }

    @Override
    public void run() throws Exception {
        if (timer_.isExpired()) {
            sub_.getGrabSubsystem().setAction(power_action_, true) ;
            sub_.getSpinSubsystem().setAction(stop_spinner_action_, true) ;
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberGrabLoadedGamepieceAction" ;
    }
}
