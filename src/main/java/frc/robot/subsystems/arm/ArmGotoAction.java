package frc.robot.subsystems.arm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderGotoAction;

public class ArmGotoAction extends Action {
    private ArmSubsystem arm_ ;
    private double lower_target_ ;
    private double upper_target_ ;
    private MotorEncoderGotoAction upper_action_ ;
    private MotorEncoderGotoAction lower_action_ ;

    public ArmGotoAction(ArmSubsystem sub, double lower, double upper) {
        super(sub.getRobot().getMessageLogger()) ;
        arm_ = sub ;

        upper_target_ = upper ;
        lower_target_ = lower ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        lower_action_ = new MotorEncoderGotoAction(arm_.getLowerSubsystem(), lower_target_, true) ;
        arm_.getLowerSubsystem().setAction(lower_action_, true) ;

        upper_action_ = new MotorEncoderGotoAction(arm_.getUpperSubsystem(), upper_target_, true) ;
        arm_.getUpperSubsystem().setAction(upper_action_, true) ;
    }

    @Override
    public void run() {
        if (upper_action_.isDone() && lower_action_.isDone()) {
            setDone() ;
        }
    }

    public String toString(int indent) {
        return spaces(indent) + "ArmGotoAction" ;
    }
}
