package frc.robot.subsystems.grabber ;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderHoldAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class GrabberShootAction extends Action {
    private GrabberSubsystem sub_ ;
    private MotorEncoderHoldAction hold_action_ ;
    private MotorEncoderPowerAction start_spinner_action_ ;
    private MotorEncoderPowerAction stop_spinner_action_ ;
    private XeroTimer start_shoot_timer_ ;
    private XeroTimer finish_timer_ ;
    private boolean shooting_ ;
    
    public GrabberShootAction(GrabberSubsystem sub) throws MissingParameterException, BadParameterTypeException{
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;

        double shootpos = sub.getSettingsValue("shoot:cone:position").getDouble() ;
        double spinpower = sub.getSettingsValue("shoot:cone:spin-power").getDouble();
        double spindelay = sub.getSettingsValue("shoot:cone:spin-delay").getDouble();
        double finishtime = sub.getSettingsValue("shoot:cone:finish-time").getDouble();

        hold_action_ = new MotorEncoderHoldAction(sub.getGrabSubsystem(), shootpos);
        start_spinner_action_ = new MotorEncoderPowerAction(sub.getSpinSubsystem(), spinpower);
        stop_spinner_action_ = new MotorEncoderPowerAction(sub.getSpinSubsystem(), 0.0);
        start_shoot_timer_ = new XeroTimer(sub.getRobot(), "start-shoot", spindelay);
        finish_timer_ = new XeroTimer(sub.getRobot(), "start-shoot", finishtime);
    }

    @Override
    public void start() throws Exception {
        super.start();

        shooting_ = false ;
        sub_.getGrabSubsystem().setDefaultAction(hold_action_);
        start_shoot_timer_.start() ;      
    }    

    @Override
    public void run() throws Exception { 
        super.run() ;

        if (shooting_) {
            if (start_shoot_timer_.isExpired()) {
                shooting_ = true ;
                sub_.getSpinSubsystem().setAction(start_spinner_action_, true) ;
                finish_timer_.start() ;
            }
        }
        else {
            if (finish_timer_.isExpired()) {
                sub_.getSpinSubsystem().setAction(stop_spinner_action_, true) ;                
            }
        }
    }    

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberShootAction" ;
    }    
}
