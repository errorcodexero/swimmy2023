package frc.robot.subsystems.arm;

import org.xero1425.base.actions.ParallelAction;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.actions.ParallelAction.DonePolicy;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.misc.MissingParameterException;

public class ArmGotoAction extends SequenceAction {
    private ArmSubsystem arm_ ;

    public ArmGotoAction(ArmSubsystem sub, String name) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;
        arm_ = sub ;

        double upper, lower ;

        int i = 1 ;
        boolean looping = true ;
        while (looping) {
            String entry = name + ":" + Integer.toString(i) ;
            try {
                lower = arm_.getSettingsValue(entry + ":lower").getDouble();
                upper = arm_.getSettingsValue(entry + ":upper").getDouble();

                addNext(lower, upper) ;
                i++;
            }
            catch(MissingParameterException ex) {
                looping = false ;
            }
        }
    }

    public ArmGotoAction(ArmSubsystem sub, double[] lower, double [] upper) throws Exception {
        super(sub.getRobot().getMessageLogger()) ;

        arm_ = sub;

        int i = 0 ;
        while (i < lower.length && i < upper.length) {
            addNext(lower[i], upper[i]) ;
        }
    }

    public ArmGotoAction(ArmSubsystem sub, double lower, double upper) throws Exception {
        super(sub.getRobot().getMessageLogger());

        arm_ = sub;
        addNext(lower, upper) ;
    }

    public String toString(int indent) {
        return spaces(indent) + "ArmGotoAction" ;
    }

    private void addNext(double lower, double upper) throws Exception {
        ParallelAction action = new ParallelAction(arm_.getRobot().getMessageLogger(), DonePolicy.All);
        MotorEncoderGotoAction lower_action = new MotorEncoderGotoAction(arm_.getLowerSubsystem(), lower, true);
        MotorEncoderGotoAction upper_action = new MotorEncoderGotoAction(arm_.getUpperSubsystem(), upper, true);
        action.addSubActionPair(arm_.getLowerSubsystem(), lower_action, true) ;
        action.addSubActionPair(arm_.getUpperSubsystem(), upper_action, true) ;
    }
}
