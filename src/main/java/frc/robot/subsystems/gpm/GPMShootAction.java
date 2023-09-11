package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;

import frc.robot.subsystems.arm.ArmStaggeredGotoMagicAction;
import frc.robot.subsystems.grabber.GrabberGrabLoadedGamepieceAction;
import frc.robot.subsystems.grabber.GrabberShootAction;

public class GPMShootAction extends Action {
    private GPMSubsystem sub_ ;
    private GrabberGrabLoadedGamepieceAction grab_cone_ ;
    private ArmStaggeredGotoMagicAction shoot_arm_action_ ;
    private GrabberShootAction shoot_grabber_action_ ;
    private boolean is_grabbed_ ;
    private boolean is_arm_in_place_ ;

    public GPMShootAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());
        
        sub_ = sub ;
        grab_cone_ = new GrabberGrabLoadedGamepieceAction(sub_.getGrabber(), "shoot");
        shoot_arm_action_ = new ArmStaggeredGotoMagicAction(sub_.getArm(), "shoot") ;
        shoot_grabber_action_ = new GrabberShootAction(sub_.getGrabber());
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        is_arm_in_place_ = false ;
        is_grabbed_ = false ;
        sub_.getGrabber().setAction(grab_cone_, true) ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (!is_grabbed_) {
            if (grab_cone_.isDone()) {
                is_grabbed_ = true ;
                sub_.getArm().setAction(shoot_arm_action_, true) ;
            }
        }
        else if (!is_arm_in_place_) {
            if (shoot_arm_action_.isDone()) {
                sub_.getGrabber().setAction(shoot_grabber_action_, true);
                is_arm_in_place_ = true ;
            }
        }
        else {
            if (shoot_grabber_action_.isDone()) {
                setDone() ;
            }
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMStowAction";
    }
}
