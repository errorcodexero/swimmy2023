package frc.robot.subsystems.grabber;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;

public class GrabberCalibrateAction extends Action {

    private final double StoppedThreshold = 50 ;

    private GrabberSubsystem sub_ ;
    private double last_value_ ;

    public GrabberCalibrateAction(GrabberSubsystem sub) {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        MotorEncoderSubsystem grab = sub_.getGrabSubsystem();
        last_value_ = grab.getPosition() ;
        grab.setAction(new MotorEncoderPowerAction(grab, -0.1));
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        MotorEncoderSubsystem grab = sub_.getGrabSubsystem();
        if (Math.abs(grab.getPosition() - last_value_) < StoppedThreshold) {
            grab.setAction(new MotorEncoderPowerAction(grab, 0.0));
            grab.postHWInit();
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "GrabberCalibrateAction" ;
    }
}
