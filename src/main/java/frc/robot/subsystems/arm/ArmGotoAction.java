package frc.robot.subsystems.arm;

import java.util.LinkedList;
import java.util.List;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MissingParameterException;

public class ArmGotoAction extends Action {
    private ArmSubsystem arm_;
    private List<Double> upper_targets_;
    private List<Double> lower_targets_;
    private int current_index_;

    private MotorEncoderGotoAction upper_action_;
    private MotorEncoderGotoAction lower_action_;

    //
    // Create an ARM goto action with a single set of upper and lower targets
    //
    public ArmGotoAction(ArmSubsystem sub, double lower, double upper) {
        this(sub.getRobot().getMessageLogger());
        arm_ = sub;

        upper_targets_.add(upper);
        lower_targets_.add(lower);
    }

    //
    // Create an ARM goto action with arrays of targets. Used mostly for testing form the test
    // automode.  Not expected to be use in actual robot code.
    //
    public ArmGotoAction(ArmSubsystem sub, double [] lower, double [] upper) throws Exception {
        this(sub.getRobot().getMessageLogger());

        if (lower.length != upper.length) {
            throw new Exception("the to arrays, upper and lower, must be the same size") ;
        }

        for(int i = 0 ; i < lower.length ; i++) {
            upper_targets_.add(upper[i]);
            lower_targets_.add(lower[i]);
        }
    }

    //
    // Create an ARM goto action with an array of targets read from the settings file.
    //
    public ArmGotoAction(ArmSubsystem sub, String value) throws BadParameterTypeException, MissingParameterException {
        this(sub.getRobot().getMessageLogger());

        int i = 1;
        current_index_ = 0;
        while (true) {
            if (sub.isSettingDefined(value + ":" + Integer.toString(i) + ":upper")
                    && sub.isSettingDefined(value + ":" + Integer.toString(i) + ":lower")) {
                upper_targets_.add(
                        sub.getSettingsValue(value + ":" + Integer.toString(i) + ":upper").getDouble());
                lower_targets_.add(
                        sub.getSettingsValue(value + ":" + Integer.toString(i) + ":lower").getDouble());
                i++;
            } else {
                break;
            }
        }

        arm_ = sub;
    }

    private ArmGotoAction(MessageLogger logger) {
        super(logger);

        //
        // Not super critical, but when you have multiple initializers,
        // put common code in a shared private constructor so there is one version of the
        // init code.
        //

        upper_targets_ = new LinkedList<Double>();
        lower_targets_ = new LinkedList<Double>();
    }

    @Override
    public void start() throws Exception {
        super.start();

        if (lower_targets_.size() == 0) {
            //
            // If we created an action with no targets, we are done immediately
            //
            setDone() ;
        }
        else {
            lower_action_ = new MotorEncoderGotoAction(arm_.getLowerSubsystem(), lower_targets_.get(current_index_), true);
            arm_.getLowerSubsystem().setAction(lower_action_, true);

            upper_action_ = new MotorEncoderGotoAction(arm_.getUpperSubsystem(), upper_targets_.get(current_index_), true);
            arm_.getUpperSubsystem().setAction(upper_action_, true);
        }

        //
        // Note I explictly set the current_index value here.  Note that start can be called multiple times and it must re-initialize the
        // action to be run again.  If we just increment, then the second time this action is used, the start call will increment past the 
        // size of the lower_targets_ and upper_targets array.
        //
        current_index_ = 1 ;

        // current_index_++;
    }

    @Override
    public void run() throws Exception {
        super.run();
        if (upper_action_.isDone() && lower_action_.isDone()) {
            if (current_index_ == upper_targets_.size()) {
                //
                // We have processed all targets, we are done
                //
                setDone();
            } else {
                //
                // The previous target is done, move to the next one
                //
                lower_action_ = new MotorEncoderGotoAction(arm_.getLowerSubsystem(),
                        lower_targets_.get(current_index_), true);
                arm_.getLowerSubsystem().setAction(lower_action_, true);

                upper_action_ = new MotorEncoderGotoAction(arm_.getUpperSubsystem(),
                        upper_targets_.get(current_index_), true);
                arm_.getUpperSubsystem().setAction(upper_action_, true);
                current_index_++;
            }
        }
    }

    public String toString(int indent) {
        return spaces(indent) + "ArmGotoAction";
    }
}
