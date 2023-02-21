package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.arm.ArmGotoAction;
import frc.robot.subsystems.grabber.GrabberGrabGampieceAction;

public class GPMStartWithGPAction extends Action {
    private GPMSubsystem sub_ ;
    private GrabberGrabGampieceAction grabber_action_ ;
    private ArmGotoAction arm_action_ ;

    public GPMStartWithGPAction(GPMSubsystem sub) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;

        grabber_action_ = new GrabberGrabGampieceAction(sub_.getGrabber());
        arm_action_ = new ArmGotoAction(sub_.getArm(), 0.0, 20000);
    }
    
    @Override
    public void start() throws Exception {
        super.start() ;
        sub_.getGrabber().setAction(grabber_action_, true);
    }

    @Override
    public void run() throws Exception {
        super.run() ;
        if (grabber_action_.isDone() && arm_action_.isDone()) {
            setDone() ;
        }
        else if (grabber_action_.isDone() && sub_.getArm().getAction() != arm_action_) {
            sub_.getArm().setAction(arm_action_, true) ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMStartWithGPAction" ;
    }
}
