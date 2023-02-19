package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;

import frc.robot.subsystems.arm.ArmGotoAction;
import frc.robot.subsystems.grabber.GrabberStartCollectAction;
import frc.robot.subsystems.grabber.GrabberStopCollectAction;

public class GPMCollectAction extends Action {

    private GPMSubsystem subsystem_;
    private GrabberStartCollectAction grabber_start_collect_action_;
    private GrabberStopCollectAction grabber_stop_collect_action_;

    private XeroTimer timer_ ;
    private boolean running_ ;

    private ArmGotoAction arm_collect_action_;
    private ArmGotoAction arm_retract_action_;

    public GPMCollectAction(GPMSubsystem subsystem, boolean ground) throws Exception {
        super(subsystem.getRobot().getMessageLogger());

        if (ground) {
            arm_collect_action_ = new ArmGotoAction(subsystem_.getArm(), "collect:extend-ground");
            arm_retract_action_ = new ArmGotoAction(subsystem_.getArm(), "collect:retract-ground");
        }
        else {
            arm_collect_action_ = new ArmGotoAction(subsystem_.getArm(), "collect:extend-shelf");
            arm_retract_action_ = new ArmGotoAction(subsystem_.getArm(), "collect:retract-shelf");
        }       

        grabber_stop_collect_action_ = new GrabberStopCollectAction(subsystem.getGrabber());
        grabber_start_collect_action_ = new GrabberStartCollectAction(subsystem_.getGrabber());

        timer_ = new XeroTimer(subsystem.getRobot(), "collect", 0.5);
    }

    @Override
    public void start() throws Exception {
        super.start();

        running_ = false ;
        subsystem_.getGrabber().setAction(grabber_start_collect_action_, true);
        subsystem_.getArm().setAction(arm_collect_action_, true);
    }

    @Override
    public void run() throws Exception {
        super.run();

        if (arm_collect_action_.isDone() && grabber_start_collect_action_.isDone()) {
            if (running_) {
                if (timer_.isExpired()) {
                    subsystem_.getArm().setAction(arm_retract_action_, true);
                    setDone() ;
                }
            }
            else if (subsystem_.getGrabber().sensor()) {
                subsystem_.getGrabber().setAction(grabber_stop_collect_action_, true);
                timer_.start() ;
                running_ = true ;
            }
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMCollectAction";
    }
}
