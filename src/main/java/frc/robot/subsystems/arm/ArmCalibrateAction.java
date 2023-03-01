package frc.robot.subsystems.arm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class ArmCalibrateAction extends Action {

    private enum State {
        Idle,
        UpperArmUp,
        LowerArmBack,
        UpperArmBack,
        Done,
        Error
    }

    private final double StoppedThreshold = 250 ;

    private ArmSubsystem sub_ ;
    private State state_ ;
    private double last_value_ ;
    private XeroTimer timer_ ;

    public ArmCalibrateAction(ArmSubsystem sub) {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;
        state_ = State.Idle;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        state_ = State.UpperArmUp;
        MotorEncoderSubsystem upper = sub_.getUpperSubsystem();
        upper.setAction(new MotorEncoderPowerAction(upper, 0.1));
        timer_ = new XeroTimer(sub_.getRobot(), "recalibrate", 1.0);
        timer_.start();
    }

    @Override 
    public void run() throws Exception {
        super.run() ;

        switch(state_) {
            case Idle:
                break ;

            case UpperArmUp:
                stateUpperArmUp() ;
                break;

            case LowerArmBack:
                stateLowerArmBack() ;
                break; 

            case UpperArmBack:
                stateUpperArmBack();
                break;

            case Done:
                break;

            case Error:
                break;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ArmCalibrateAction";
    }

    private void stateUpperArmUp() {
        if (timer_.isExpired()) {
            MotorEncoderSubsystem upper = sub_.getUpperSubsystem();
            upper.setAction(new MotorEncoderPowerAction(upper, 0.0));

            MotorEncoderSubsystem lower = sub_.getLowerSubsystem();
            lower.setAction(new MotorEncoderPowerAction(lower, -0.1));
            last_value_ = lower.getPosition();

            state_ = State.LowerArmBack;
        }
    }

    private void stateLowerArmBack() {
        MotorEncoderSubsystem lower = sub_.getLowerSubsystem();
        if (Math.abs(lower.getPosition() - last_value_) < StoppedThreshold) {
            lower.setAction(new MotorEncoderPowerAction(lower, 0.0));
            try {
                lower.postHWInit();
            }
            catch(Exception ex) {
                MessageLogger logger = sub_.getRobot().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("ArmCalibrateAction: lower: postHWInit failed - " + ex.getMessage());
                logger.endMessage();
                setDone() ;
                state_ = State.Error ;
            }

            MotorEncoderSubsystem upper = sub_.getUpperSubsystem();
            upper.setAction(new MotorEncoderPowerAction(upper, -0.1));
            state_ = State.UpperArmBack;
            last_value_ = upper.getPosition();
        }
    }

    private void stateUpperArmBack() {
        MotorEncoderSubsystem upper = sub_.getUpperSubsystem();
        if (Math.abs(upper.getPosition() - last_value_) < StoppedThreshold) {
            upper.setAction(new MotorEncoderPowerAction(upper, 0.0));
            try {
                upper.postHWInit();
            }
            catch(Exception ex) {
                MessageLogger logger = sub_.getRobot().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("ArmCalibrateAction: upper: postHWInit failed - " + ex.getMessage());
                logger.endMessage();
                setDone() ;
                state_ = State.Error ;
            }

            state_ = State.Done ;
            setDone() ;
        }
    }
}
