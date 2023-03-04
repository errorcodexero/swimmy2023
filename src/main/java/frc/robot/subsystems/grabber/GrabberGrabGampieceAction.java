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
    private boolean ground_ ;

    public GrabberGrabGampieceAction(GrabberSubsystem sub, RobotOperation.GamePiece gp, boolean ground) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;
        ground_ = ground;

        double holdpos = 0 ;
        double spinpower = 0;
        double spindelay = 0 ;

        if (ground) {
            if (gp == RobotOperation.GamePiece.Cone)
                holdpos = sub.getSettingsValue("close:ground:position:cone").getDouble() ;
            else
                holdpos = sub.getSettingsValue("close:ground:position:cube").getDouble() ;

            spinpower = sub.getSettingsValue("close:ground:spin-power").getDouble();
            spindelay = sub.getSettingsValue("close:ground:spin-delay").getDouble();
        }
        else {
            if (gp == RobotOperation.GamePiece.Cone)
                holdpos = sub.getSettingsValue("close:shelf:position:cone").getDouble() ;
            else
                holdpos = sub.getSettingsValue("close:shelf:position:cube").getDouble() ;

            spinpower = sub.getSettingsValue("close:shelf:spin-power").getDouble();
            spindelay = sub.getSettingsValue("close:shelf:spin-delay").getDouble();
        }

        hold_action_ = new MotorEncoderHoldAction(sub.getGrabSubsystem(), holdpos);
        power_action_ = new MotorEncoderPowerAction(sub.getGrabSubsystem(), sub.getSettingsValue("close:hold-power").getDouble());
        start_spinner_action_ = new MotorEncoderPowerAction(sub.getSpinSubsystem(), spinpower);

        stop_spinner_action_ = new MotorEncoderPowerAction(sub.getSpinSubsystem(), 0.2);

        timer_ = new XeroTimer(sub.getRobot(), "grabbergp", spindelay);
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
        return spaces(indent) + "GrabberGrabGampieceAction(" + (ground_ ? "ground" : "shelf") + ")";
    }
}
