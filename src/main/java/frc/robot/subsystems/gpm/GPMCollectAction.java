package frc.robot.subsystems.gpm;

import java.util.concurrent.LinkedBlockingDeque;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.arm.ArmGotoAction;
import frc.robot.subsystems.grabber.GrabberCloseAction;
import frc.robot.subsystems.grabber.GrabberOpenAction;

public class GPMCollectAction extends Action {

    private enum State {
        Deploying1,
        Waiting,
        Retracting1,
        Retracting2,
        Retracting3,
    } ;

    private State state_ ;

    private GPMSubsystem sub_ ;

    private ArmGotoAction extend_arm_ ;
    private ArmGotoAction retract_arm1_ ;
    private ArmGotoAction retract_arm2_ ;
    private ArmGotoAction retract_arm3_ ;
    private GrabberOpenAction open_grabber_ ;
    private GrabberCloseAction close_grabber_ ;

    public GPMCollectAction(GPMSubsystem sub) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;

        double v1 = sub_.getSettingsValue("collect:extend:lower").getInteger() ;
        double v2= sub_.getSettingsValue("collect:extend:upper").getInteger() ;

        extend_arm_ = new ArmGotoAction(sub_.getArm(), v1, v2);

        v1 = sub_.getSettingsValue("collect:retract1:lower").getInteger() ;
        v2= sub_.getSettingsValue("collect:retract1:upper").getInteger() ;
        retract_arm1_ = new ArmGotoAction(sub_.getArm(), v1, v2) ;

        v1 = sub_.getSettingsValue("collect:retract2:lower").getInteger() ;
        v2= sub_.getSettingsValue("collect:retract2:upper").getInteger() ;
        retract_arm2_ = new ArmGotoAction(sub_.getArm(), v1, v2) ;

        retract_arm3_ = new ArmGotoAction(sub_.getArm(), 0, 0) ;

        v1 = sub_.getSettingsValue("collect:grabber:power").getDouble();
        open_grabber_ = new GrabberOpenAction(sub_.getGrabber(), v1);

        v1 = sub_.getSettingsValue("collect:grabber:closedelay").getDouble();
        close_grabber_ = new GrabberCloseAction(sub_.getGrabber(), v1);

        v1 = sub_.getSettingsValue("collect:grabber:power").getDouble();
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        state_ = State.Deploying1 ;
        sub_.getArm().setAction(extend_arm_, true) ;
        sub_.getGrabber().setAction(open_grabber_, true);
    }

    @Override
    public void run() throws Exception {
        super.run() ;
        switch(state_) {
            case Deploying1:
                if (extend_arm_.isDone() && open_grabber_.isDone()) {
                    state_ = State.Waiting ;
                }
                break; 

            case Waiting:
                if (sub_.getGrabber().isSensorActive()) {
                    sub_.getGrabber().setAction(close_grabber_, true) ;
                    sub_.getArm().setAction(retract_arm1_, true) ;
                    state_ = State.Retracting1 ;
                }
                break ;

            case Retracting1:
                if (retract_arm1_.isDone() && close_grabber_.isDone()) {
                    sub_.getArm().setAction(retract_arm2_, true) ;
                    state_ = State.Retracting2;
                }
                break; 

            case Retracting2:
                if (retract_arm2_.isDone()) {
                    sub_.getArm().setAction(retract_arm3_, true) ;
                    state_ = State.Retracting3;
                }
                break;

            case Retracting3:
                if (retract_arm3_.isDone()) {
                    setDone() ;
                }
                break;
            }
    }

    public String toString(int indent) {
        return spaces(indent) + "GPMCollectAction" ;
    }
}
