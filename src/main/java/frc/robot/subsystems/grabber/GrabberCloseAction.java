package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerAction;

public class GrabberCloseAction extends Action {

    private GrabberSubsystem sub_ ;
    private MotorPowerAction left_ ;
    private MotorPowerAction right_ ;
    private XeroTimer timer_ ;

    public GrabberCloseAction(GrabberSubsystem sub, double delay) {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;

        left_ = new MotorPowerAction(sub_.getLeftSubsystem(), 0.0) ;
        right_ = new MotorPowerAction(sub_.getRightSubsystem(), 0.0) ;

        timer_ = new XeroTimer(sub.getRobot(), "grabber-close-timer", delay);
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        
        sub_.close() ;
        timer_.start() ;

        
        setDone() ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (timer_.isExpired()) {
            sub_.getLeftSubsystem().setAction(left_, true) ;
            sub_.getRightSubsystem().setAction(right_, true) ;
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberCloseAction" ;
    }
}


