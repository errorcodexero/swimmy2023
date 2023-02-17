package frc.robot.subsystems.gpm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.arm.ArmGotoAction;
import frc.robot.subsystems.grabber.GrabberStowAction;
import frc.robot.subsystems.toplevel.RobotOperation;

public class GPMPlaceAction extends Action {

    private enum State {
        Idle,
        ExtendingArm,
        DroppingGamepiece,
        RetractingArm,
    }

    private State state_ ;
    private GPMSubsystem sub_ ;
    private ArmGotoAction arm_extend_action_ ;
    private ArmGotoAction arm_retract_action_ ;
    private GrabberStowAction grabber_drop_item_ ;
    private XeroTimer timer_ ;

    public GPMPlaceAction(GPMSubsystem sub, RobotOperation.Location loc, RobotOperation.GamePiece gp) throws MissingParameterException, BadParameterTypeException {
        super(sub.getRobot().getMessageLogger());

        state_ = State.Idle ;
        sub_ = sub ;

        String armpos = "place:" ;

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
        armpos += ":extend";

        arm_extend_action_ = new ArmGotoAction(sub_.getArm(), armpos + ":extend");
        arm_retract_action_ = new ArmGotoAction(sub_.getArm(), armpos + ":extend");

        double duration = sub.getSettingsValue("place-delay").getDouble();
        timer_ = new XeroTimer(sub.getRobot(), "placeaction", duration) ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getArm().setAction(arm_extend_action_, true) ;
        state_ = State.ExtendingArm ;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        switch(state_) {
            case Idle:
                break;

            case ExtendingArm:
                if (arm_extend_action_.isDone()) {
                    sub_.getGrabber().setAction(grabber_drop_item_, true);
                    state_ = State.DroppingGamepiece;
                    timer_.start();
                }
                break;

            case DroppingGamepiece:
                if (timer_.isExpired()) {
                    sub_.getArm().setAction(arm_retract_action_);
                    state_ = State.RetractingArm ;
                }
                break ;

            case RetractingArm:
                if (arm_retract_action_.isDone()) {
                    state_ = State.Idle ;
                    setDone() ;
                }
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GPMStowAction";
    }
}
