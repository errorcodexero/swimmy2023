package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.arm.ArmStaggeredGotoAction;
import frc.robot.subsystems.arm.ArmStaggeredGotoMagicAction;
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
    private ArmStaggeredGotoMagicAction arm_extend_action_ ;
    private ArmStaggeredGotoMagicAction arm_retract_action_ ;
    private GrabberStowAction grabber_drop_item_ ;
    private MotorEncoderPowerAction shoot_action_ ;
    private boolean ready_to_drop_ ;
    private boolean drop_game_piece_ ;
    private XeroTimer drop_timer_ ;
    private String title_ ;
    private boolean is_dropped_ ;
    private boolean extend_arm_ ;
    private PlaceMethod place_method_ ;
    private XeroTimer force_drop_timer_ ;

    public GPMPlaceAction(GPMSubsystem sub, RobotOperation.Location loc, RobotOperation.GamePiece gp, boolean force, boolean still) throws Exception {
        super(sub.getRobot().getMessageLogger());

        state_ = State.Idle ;
        sub_ = sub ;
        String armpos ;

        place_method_ = PlaceMethod.Drop ;
        if (still) {
            //
            // We can change place: to place-still: and put a different set of
            // values in the settings JSON file for placing when we are still.  After moving to
            // motion magic, try to go back to a single set of values first.
            //
            armpos = "place:" ;
            force_drop_timer_ = new XeroTimer(sub_.getRobot(), "still-place-settling", 0.05) ;
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
        arm_extend_action_ = new ArmStaggeredGotoMagicAction(sub_.getArm(), armpos + ":extend");
        arm_retract_action_ = new ArmStaggeredGotoMagicAction(sub_.getArm(), armpos + ":retract");

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
                    if (force_drop_timer_ != null) {
                        force_drop_timer_.start() ;
                    }
                }
                break;

            case WaitingToDrop:
                if (place_method_ == PlaceMethod.Drop) {
                    if (drop_game_piece_ || (force_drop_timer_ != null && force_drop_timer_.isExpired())) {
                        sub_.getGrabber().setAction(grabber_drop_item_, true);
                        drop_timer_.start();
                        state_ = State.DroppingGamepiece;
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
