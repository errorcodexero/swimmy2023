package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import frc.robot.subsystems.grabber.GrabberStartCollectAction;
import frc.robot.subsystems.arm.ArmStaggeredGotoAction;
import frc.robot.subsystems.grabber.GrabberGrabGampieceAction;
import frc.robot.subsystems.grabber.GrabberStowAction;
import frc.robot.subsystems.toplevel.RobotOperation;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;

public class GPMCollectAction extends Action {

    private enum State {
        Idle,
        WaitingForDeploy,
        WaitingForSensor,
        CloseGrabber,
        StowGrabber,
        LiftArm,
        RetractArm,
        Done
    } ;

    private GPMSubsystem subsystem_;
    private GrabberStartCollectAction grabber_start_collect_action_;
    private GrabberGrabGampieceAction grabber_stop_collect_action_;
    private GrabberStowAction grabber_stow_action_ ;

    private boolean timer_active_ ;
    private Action act_ ;
    private XeroTimer timer_ ;
    private XeroTimer ground_done_timer_ ;

    private ArmStaggeredGotoAction arm_collect_action_ ;
    private ArmStaggeredGotoAction arm_lift_action_;
    private ArmStaggeredGotoAction arm_retract_action_;

    private State state_ ;
    private boolean ground_ ;

    public GPMCollectAction(GPMSubsystem subsystem, RobotOperation.GamePiece gp, boolean ground) throws Exception {
        super(subsystem.getRobot().getMessageLogger());

        subsystem_ = subsystem;
        ground_ = ground ;

        if (ground) {
            arm_collect_action_ = new ArmStaggeredGotoAction(subsystem_.getArm(), "collect:extend-ground", false);
            arm_retract_action_ = new ArmStaggeredGotoAction(subsystem_.getArm(), "collect:retract-ground", false);
            arm_lift_action_ = null ;
        }
        else {
            arm_collect_action_ = new ArmStaggeredGotoAction(subsystem_.getArm(), "collect:extend-shelf", false);
            arm_retract_action_ = new ArmStaggeredGotoAction(subsystem_.getArm(), "collect:retract-shelf", false);
            arm_lift_action_ = new ArmStaggeredGotoAction(subsystem.getArm(), "collect:lift-shelf", false) ;
        }       

        grabber_stop_collect_action_ = new GrabberGrabGampieceAction(subsystem.getGrabber(), gp, ground);
        grabber_start_collect_action_ = new GrabberStartCollectAction(subsystem_.getGrabber(), gp);
        grabber_stow_action_ = new GrabberStowAction(subsystem_.getGrabber());

        ground_done_timer_ = new XeroTimer(subsystem.getRobot(), "groundtimer", 0.1);

        timer_ = null ;
        state_ = State.Idle ;
    }

    public GPMCollectAction(GPMSubsystem subsystem, RobotOperation.GamePiece gp, boolean ground, Action act, double timeout) throws Exception {
        this(subsystem, gp, ground) ;
        act_ = act ;
        timer_ = new XeroTimer(subsystem.getRobot(), "collect-overall", timeout);
    }

    public void setGamePiece(GamePiece gp) {
        try {
            grabber_start_collect_action_ = new GrabberStartCollectAction(subsystem_.getGrabber(), gp);
            subsystem_.getGrabber().setAction(grabber_start_collect_action_, true);
        }
        catch(Exception ex) {

        }
    }

    @Override
    public void start() throws Exception {
        super.start();

        timer_active_ = false ;
        state_ = State.WaitingForDeploy ;

        subsystem_.getGrabber().setAction(grabber_start_collect_action_, true);
        subsystem_.getArm().setAction(arm_collect_action_, true);
    }

    private boolean isCollectDone() {
        MessageLogger logger = subsystem_.getRobot().getMessageLogger();
        boolean ret = false;

        if (subsystem_.getGrabber().sensor()) {
            ret = true ;
            logger.startMessage(MessageType.Info).add("GPMCollectAction - sensor detected").endMessage();
        }
        else if (timer_ != null && timer_.isExpired()) {
            ret = true ;
            logger.startMessage(MessageType.Info).add("GPMCollectAction - timer timed out").endMessage();
        }

        return ret ;
    }

    @Override
    public void run() throws Exception {
        super.run();
        
        if (act_ != null && act_.isDone() && timer_active_ == false) {
            timer_.start() ;
            timer_active_ = true ;
        }

        State orig = state_ ;
        switch(state_) {
            case Idle:
                break ;

            case WaitingForDeploy:
                if (arm_collect_action_.isDone() && grabber_start_collect_action_.isDone()) {
                    if (timer_ != null && act_ == null) {
                        timer_.start() ;
                    }
                    state_ = State.WaitingForSensor ;
                }
                break ;

            case WaitingForSensor:
                if (isCollectDone()) {
                    subsystem_.getGrabber().setAction(grabber_stop_collect_action_, true);
                    state_ = State.CloseGrabber;
                }
                break ;

            case CloseGrabber:
                processStowOrCloseGrabber(grabber_stop_collect_action_) ;
                break ;

            case StowGrabber:
                processStowOrCloseGrabber(grabber_stow_action_) ;
                break ;

            case LiftArm:
                if (arm_lift_action_.isDone()) {
                    state_ = State.Done ;
                    setDone() ;
                }
                break ;

            case RetractArm:
                if (ground_ && ground_done_timer_.isExpired()) {
                    state_ = State.Done ;
                    setDone() ;
                }
                else if (arm_retract_action_.isDone()) {
                    state_ = State.Done ;
                    setDone() ;
                }
                break ;

            case Done:
                break ;
        }

        if (state_ != orig) {
            MessageLogger logger = subsystem_.getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, subsystem_.getLoggerID());
            logger.add("State Changed " + orig.toString() + " -> " + state_.toString()) ;
            logger.endMessage();
        }
    }

    private void processStowOrCloseGrabber(Action action) {
        if (action.isDone()) {
            if (ground_) {
                ground_done_timer_.start() ;
                subsystem_.getArm().setAction(arm_retract_action_, true);
                state_ = State.RetractArm ;
            }
            else {
                subsystem_.getArm().setAction(arm_lift_action_, true);
                state_ = State.LiftArm ;
            }
        }        
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMCollectAction(" + (ground_ ? "ground" : "shelf") ;
    }
}
