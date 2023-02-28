package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderHoldAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.toplevel.RobotOperation;

public class GrabberGrabGampieceAction extends Action {
    private GrabberSubsystem sub_ ;
    private XeroTimer timer_;
    private MotorEncoderHoldAction hold_action_ ;
    private MotorEncoderPowerAction power_action_ ;
    private MotorEncoderPowerAction start_spinner_action_ ;
    private MotorEncoderPowerAction stop_spinner_action_ ;

    public GrabberGrabGampieceAction(GrabberSubsystem sub, RobotOperation.GamePiece gp) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;

        double v = 0 ;
        
        if (gp == RobotOperation.GamePiece.Cone)
            v = sub.getSettingsValue("close:cone-position").getDouble() ;
        else
            v = sub.getSettingsValue("close:cube-position").getDouble() ;

        hold_action_ = new MotorEncoderHoldAction(sub.getGrabSubsystem(), v);
        power_action_ = new MotorEncoderPowerAction(sub.getGrabSubsystem(), 0.1);
        stop_spinner_action_ = new MotorEncoderPowerAction(sub.getSpinSubsystem(), 0.1);

        v = sub.getSettingsValue("close:spin-power").getDouble() ;
        start_spinner_action_ = new MotorEncoderPowerAction(sub.getSpinSubsystem(), v);

        v = sub.getSettingsValue("close:delay").getDouble() ;
        timer_ = new XeroTimer(sub.getRobot(), "grabbergp", v) ;
    }

    @Override
    public void start() throws Exception {
        super.start();
        
        sub_.getSpinSubsystem().setAction(start_spinner_action_, true) ;
        sub_.getGrabSubsystem().setDefaultAction(hold_action_);
        timer_.start() ;
    }

    @Override
    public void run() throws Exception { 
        super.run();
        
        if (timer_.isExpired()) {
            sub_.getGrabSubsystem().setAction(power_action_, true) ;
            sub_.getSpinSubsystem().setAction(stop_spinner_action_, true) ;
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberGrabGampieceAction";
    }
}
