package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.arm.ArmStaggeredGotoAction;
import frc.robot.subsystems.grabber.GrabberStowAction;
import frc.robot.subsystems.toplevel.RobotOperation;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class GPMPlaceAction extends Action {

    private enum State {
        Idle,
        ExtendingArm,
        WaitingToDrop,
        DroppingGamepiece,
        ShootingGamepiece,
        RetractingArm,
    }

    private enum PlaceMethod {
        Drop,
        Shoot
    }        

    private State state_ ;
    private GPMSubsystem sub_ ;
    private ArmStaggeredGotoAction arm_extend_action_ ;
    private ArmStaggeredGotoAction arm_retract_action_ ;
    private GrabberStowAction grabber_drop_item_ ;
    private MotorEncoderPowerAction shoot_action_ ;
    private boolean ready_to_drop_ ;
    private boolean drop_game_piece_ ;
    private boolean force_drop_ ;
    private XeroTimer drop_timer_ ;
    private String title_ ;
    private boolean is_dropped_ ;
    private boolean extend_arm_ ;
    private PlaceMethod place_method_ ;

    public GPMPlaceAction(GPMSubsystem sub, RobotOperation.Location loc, RobotOperation.GamePiece gp, boolean force, boolean still) throws MissingParameterException, BadParameterTypeException {
        super(sub.getRobot().getMessageLogger());

        state_ = State.Idle ;
        sub_ = sub ;
        force_drop_ = force;
        String armpos ;

        place_method_ = PlaceMethod.Drop ;
        if (still) {
            armpos = "place-still:" ;
        }
        else {
            armpos = "place:" ;
        }

        switch(loc) {
            case Bottom:
                armpos += "bottom" ;
                break ;

            case Middle:
                armpos += "middle" ;
                break ;

            case Top:
                armpos += "top" ;
                break ;
        }

        if (gp == RobotOperation.GamePiece.Cone) {
            armpos += ":cone" ;
        }
        else {
            armpos += ":cube" ;
        }

        title_ = armpos ;
        arm_extend_action_ = new ArmStaggeredGotoAction(sub_.getArm(), armpos + ":extend", false);
        arm_retract_action_ = new ArmStaggeredGotoAction(sub_.getArm(), armpos + ":retract", false);

        double drop_duration = sub.getSettingsValue("place-delay").getDouble();
        drop_timer_ = new XeroTimer(sub.getRobot(), "place", drop_duration);

        if (gp == GamePiece.Cube && loc == Location.Bottom) {
            extend_arm_ = false;
        } else {
            extend_arm_ = true;
        }

        grabber_drop_item_ = new GrabberStowAction(sub.getGrabber());
    }

    public boolean isReadyToDrop() {
        return ready_to_drop_ ;
    }

    public void dropGamePiece() {
        drop_game_piece_ = true ;
    }

    public boolean isDropComplete() {
        return is_dropped_ ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        ready_to_drop_ = false ;
        drop_game_piece_ = false ;
        is_dropped_ = false ;

        if (extend_arm_) {
            sub_.getArm().setAction(arm_extend_action_, true) ;
            state_ = State.ExtendingArm ;
        } else {
            state_ = State.WaitingToDrop ;
            ready_to_drop_ = true ;
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        State orig = state_ ;

        switch(state_) {
            case Idle:
                break;

            case ExtendingArm:
                if (arm_extend_action_.isDone()) {
                    state_ = State.WaitingToDrop;
                    ready_to_drop_ = true ;
                }
                break;

            case WaitingToDrop:
                if (place_method_ == PlaceMethod.Drop) {
                    if (drop_game_piece_ || force_drop_) {
                        sub_.getGrabber().setAction(grabber_drop_item_, true);
                        drop_timer_.start();
                        state_ = State.DroppingGamepiece;
                    }
                } else {
                    if (drop_game_piece_ || force_drop_) {
                        sub_.getGrabber().getSpinSubsystem().setAction(shoot_action_, true) ;
                        state_ = State.ShootingGamepiece;
                    }
                }
                break;

            case DroppingGamepiece:
                if (drop_timer_.isExpired()) {
                    sub_.getArm().setAction(arm_retract_action_, true);
                    is_dropped_ = true ;
                    state_ = State.RetractingArm ;

                    setDone() ;
                }
                break ;

            case ShootingGamepiece:
                if (shoot_action_.isDone()) {
                    sub_.getArm().setAction(arm_retract_action_, true);
                    sub_.getGrabber().setAction(grabber_drop_item_, true);
                    is_dropped_ = true ;
                    state_ = State.RetractingArm ;

                    setDone() ;
                }

            case RetractingArm:
                if (arm_retract_action_.isDone()) {
                    state_ = State.Idle ;
                    setDone() ;
                }
                break;
        }

        if (orig != state_) {
            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Debug, sub_.getLoggerID());
            logger.add("GPM - ");
            logger.add(orig.toString() + " -> " + state_.toString());
            logger.endMessage();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMPlaceAction(" + title_ + ")" ;
    }
}
