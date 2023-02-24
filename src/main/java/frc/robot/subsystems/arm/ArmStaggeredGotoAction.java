package frc.robot.subsystems.arm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderGotoAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.TrapezoidalProfileConfig;

public class ArmStaggeredGotoAction extends Action {
    private ArmSubsystem sub_ ;

    private double lower_start_time_ ;
    private TrapezoidalProfileConfig lower_trap_config_ ;
    private MotorEncoderGotoAction lower_goto_ ;
    private double lower_target_ ;

    private double upper_start_time_ ;
    private TrapezoidalProfileConfig upper_trap_config_ ;
    private MotorEncoderGotoAction upper_goto_ ;
    private double upper_target_ ;

    private String key_ ;

    private double start_time_ ;

    public ArmStaggeredGotoAction(ArmSubsystem sub, String key) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub;
        key_ = key ;

        setupTrapezoidal();
    }

    private void setupTrapezoidal() throws BadParameterTypeException, MissingParameterException {
        double maxa, maxv ;

        maxa = sub_.getSettingsValue(key_ + ":lower:config:maxa").getDouble();
        maxv = sub_.getSettingsValue(key_ + ":lower:config:maxv").getDouble();
        lower_trap_config_ = new TrapezoidalProfileConfig(maxa, -maxa, maxv);

        lower_start_time_ = sub_.getSettingsValue(key_ + ":lower:delay").getDouble();
        lower_target_ = sub_.getSettingsValue(key_ + ":lower:target").getDouble();

        maxa = sub_.getSettingsValue(key_ + ":upper:config:maxa").getDouble();
        maxv = sub_.getSettingsValue(key_ + ":upper:config:maxv").getDouble();
        upper_trap_config_ = new TrapezoidalProfileConfig(maxa, -maxa, maxv);

        upper_start_time_ = sub_.getSettingsValue(key_ + ":upper:delay").getDouble();
        upper_target_ = sub_.getSettingsValue(key_ + ":lower:target").getDouble();

        lower_goto_ = null ;
        upper_goto_ = null ;
    }

    @Override
    public void start() {
        start_time_ = sub_.getRobot().getTime();
    }

    @Override
    public void run() throws Exception {
        double delta = sub_.getRobot().getTime() - start_time_ ;

        if (delta > upper_start_time_ && upper_goto_ == null) {
            upper_goto_ = new MotorEncoderGotoAction(sub_.getUpperSubsystem(), upper_target_, upper_trap_config_, true) ;
            sub_.getUpperSubsystem().setAction(upper_goto_, true);
        }

        if (delta > lower_start_time_ && lower_goto_ == null) {
            lower_goto_ = new MotorEncoderGotoAction(sub_.getUpperSubsystem(), lower_target_, lower_trap_config_, true);
            sub_.getLowerSubsystem().setAction(lower_goto_, true);
        }

        if (upper_goto_ != null && upper_goto_.isDone() && lower_goto_ != null && lower_goto_.isDone()) {
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ArmStaggeredGotoAction(" + key_ + ")";
    }
}
