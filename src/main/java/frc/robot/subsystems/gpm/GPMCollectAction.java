package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;

import frc.robot.subsystems.grabber.GrabberStartCollectAction;
import frc.robot.subsystems.arm.ArmStaggeredGotoAction;
import frc.robot.subsystems.grabber.GrabberGrabGampieceAction;
import frc.robot.subsystems.grabber.GrabberStowAction;
import frc.robot.subsystems.toplevel.RobotOperation;

public class GPMCollectAction extends Action {

    private GPMSubsystem subsystem_;
    private GrabberStartCollectAction grabber_start_collect_action_;
    private GrabberGrabGampieceAction grabber_stop_collect_action_;
    private GrabberStowAction grabber_stow_action_ ;

    private XeroTimer timer_ ;
    private boolean running_ ;

    private XeroTimer overall_ ;

    private ArmStaggeredGotoAction arm_collect_action_ ;
    private ArmStaggeredGotoAction arm_retract_action_;

    public GPMCollectAction(GPMSubsystem subsystem, RobotOperation.GamePiece gp, boolean ground) throws Exception {
        super(subsystem.getRobot().getMessageLogger());

        subsystem_ = subsystem;

        if (ground) {
            arm_collect_action_ = new ArmStaggeredGotoAction(subsystem_.getArm(), "collect:extend-ground");
            arm_retract_action_ = new ArmStaggeredGotoAction(subsystem_.getArm(), "collect:retract-ground");
        }
        else {
            arm_collect_action_ = new ArmStaggeredGotoAction(subsystem_.getArm(), "collect:extend-shelf");
            arm_retract_action_ = new ArmStaggeredGotoAction(subsystem_.getArm(), "collect:retract-shelf");
        }       

        grabber_stop_collect_action_ = new GrabberGrabGampieceAction(subsystem.getGrabber(), gp);
        grabber_start_collect_action_ = new GrabberStartCollectAction(subsystem_.getGrabber());
        grabber_stow_action_ = new GrabberStowAction(subsystem_.getGrabber());

        timer_ = new XeroTimer(subsystem.getRobot(), "collect", 0.5);
    }

    public GPMCollectAction(GPMSubsystem subsystem, RobotOperation.GamePiece gp, boolean ground, double overall) throws Exception {
        this(subsystem, gp, ground) ;
        overall_ = new XeroTimer(subsystem.getRobot(), "collect-overall", overall);
    }

    @Override
    public void start() throws Exception {
        super.start();

        running_ = false ;
        subsystem_.getGrabber().setAction(grabber_start_collect_action_, true);
        subsystem_.getArm().setAction(arm_collect_action_, true);

        if (overall_ != null) {
            overall_.start();
        }
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

        if (overall_ != null && overall_.isExpired()) {
            subsystem_.getGrabber().setAction(grabber_stow_action_, true);
            subsystem_.getArm().setAction(arm_retract_action_, true);
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMCollectAction";
    }
}
