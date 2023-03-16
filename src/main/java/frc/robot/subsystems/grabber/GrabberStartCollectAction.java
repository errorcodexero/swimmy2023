package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderHoldAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;

public class GrabberStartCollectAction extends Action {

    private GrabberSubsystem sub_ ;
    private MotorEncoderHoldAction hold_action_ ;
    private MotorEncoderPowerAction spin_action_ ;
    private GamePiece gp_ ;

    public GrabberStartCollectAction(GrabberSubsystem sub, GamePiece gp) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub ;
        gp_ = gp ;

        if (gp_ == GamePiece.Cone) {
            hold_action_ = new MotorEncoderHoldAction(sub_.getGrabSubsystem(), "positions:open:cone");
            spin_action_ = new MotorEncoderPowerAction(sub_.getSpinSubsystem(), "power:spin:cone");
        }
        else {
            hold_action_ = new MotorEncoderHoldAction(sub_.getGrabSubsystem(), "positions:open:cube");
            spin_action_ = new MotorEncoderPowerAction(sub_.getSpinSubsystem(), "power:spin:cube");
        }
    }

    @Override
    public void start() throws Exception {
        super.start();
        sub_.getGrabSubsystem().setAction(hold_action_, true);
        sub_.getSpinSubsystem().setAction(spin_action_, true);
        setDone() ;
    }

    @Override
    public void run() throws Exception {
        super.run();
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberStartCollectAction: " + ((gp_ == GamePiece.Cone) ? "Cone" : "Cube");
    }
}