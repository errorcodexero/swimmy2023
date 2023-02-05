package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerAction;

public class GrabberOpenAction extends Action {

    private GrabberSubsystem sub_ ;
    private MotorPowerAction left_ ;
    private MotorPowerAction right_ ;
    
    public GrabberOpenAction(GrabberSubsystem sub, double power) {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;

        left_ = new MotorPowerAction(sub_.getLeftSubsystem(), power) ;
        right_ = new MotorPowerAction(sub_.getRightSubsystem(), power) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.open() ;
        sub_.getLeftSubsystem().setAction(left_, true) ;
        sub_.getRightSubsystem().setAction(right_, true) ;
        
        setDone() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberOpenAction" ;
    }
}
