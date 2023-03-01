package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;

import frc.robot.subsystems.arm.ArmGotoAction;
import frc.robot.subsystems.grabber.GrabberStowAction;

public class GPMStowAction extends Action {
    private GPMSubsystem sub_ ;
    private ArmGotoAction stow_arm_action_ ;
    private GrabberStowAction stow_grabber_action_ ;

    public GPMStowAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());
        
        sub_ = sub ;
        stow_arm_action_ = new ArmGotoAction(sub_.getArm(), new double []{ 0.0, 0.0 }, new double [] { 20000.0,0.0 }) ;
        stow_grabber_action_ = new GrabberStowAction(sub_.getGrabber());
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getArm().setAction(stow_arm_action_, true) ;
        sub_.getGrabber().setAction(stow_grabber_action_, true);
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (stow_arm_action_.isDone()) {
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMStowAction";
    }
}
