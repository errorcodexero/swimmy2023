package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class GrabberStopCollectAction extends Action {

    private GrabberSubsystem sub_ ;
    private XeroTimer spin_timer_ ;
    private boolean timer_done_ ;
    private double hold_power_ ;

    public GrabberStopCollectAction(GrabberSubsystem sub) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;

        double delay = sub_.getSettingsValue("close:delay").getDouble();
        spin_timer_ = new XeroTimer(sub.getRobot(),"grabber-top-collect", delay);
        timer_done_ = false ;

        hold_power_ = sub.getSettingsValue("close:hold-power").getDouble();
    }

    @Override
    public void start() throws Exception {
        super.start();

        timer_done_ = false ;
        spin_timer_.start() ;
        sub_.getGrabSubsystem().setAction(new MotorEncoderPowerAction(sub_.getGrabSubsystem(), hold_power_), true);
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (spin_timer_.isExpired()) {
            sub_.getSpinSubsystem().setAction(new MotorEncoderPowerAction(sub_.getSpinSubsystem(), 0), true);
            timer_done_ = true ;
        }

        if (timer_done_ && sub_.getGrabSubsystem().getAction().isDone()) {
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberStopCollectAction" ;
    }
}