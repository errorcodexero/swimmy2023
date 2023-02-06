package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;

import frc.robot.subsystems.arm.ArmGotoAction;
import frc.robot.subsystems.grabber.GrabberCloseAction;
import frc.robot.subsystems.grabber.GrabberOpenAction;

public class GPMCollectAction extends Action {

    private enum State {
        Deploying,
        WaitingForSensor,
        Retracting,
        Done
    } ;

    private State state_ ;

    private GPMSubsystem sub_ ;

    private ArmGotoAction extend_arm_ ;
    private ArmGotoAction retract_arm_ ;
    private GrabberOpenAction open_grabber_ ;
    private GrabberCloseAction close_grabber_ ;

    public GPMCollectAction(GPMSubsystem sub) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;

        extend_arm_ = new ArmGotoAction(sub_.getArm(), "collect:extend") ;
        retract_arm_ = new ArmGotoAction(sub_.getArm(), "collect:retract");

        open_grabber_ = new GrabberOpenAction(sub_.getGrabber());
        close_grabber_ = new GrabberCloseAction(sub_.getGrabber());
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        state_ = State.Deploying ;
        sub_.getArm().setAction(extend_arm_, true) ;
        sub_.getGrabber().setAction(open_grabber_, true);
    }

    @Override
    public void run() throws Exception {
        super.run() ;
        switch(state_) {
            case Deploying:
                if (extend_arm_.isDone() && open_grabber_.isDone()) {
                    state_ = State.WaitingForSensor ;
                }
                break; 

            case WaitingForSensor:
                if (sub_.getGrabber().isSensorActive()) {
                    sub_.getGrabber().setAction(close_grabber_, true) ;
                    sub_.getArm().setAction(retract_arm_, true) ;
                    state_ = State.Retracting ;
                }
                break ;

            case Retracting:
                if (retract_arm_.isDone() && close_grabber_.isDone()) {
                    setDone() ;
                    state_ = State.Done;
                }
                break; 

            case Done:
                break;
        }
    }

    public String toString(int indent) {
        return spaces(indent) + "GPMCollectAction" ;
    }
}
