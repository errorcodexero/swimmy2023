package frc.robot.subsystems.arm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderMotionMagicAction;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderSubsystem;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderMotionMagicAction.HoldType;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class ArmStaggeredGotoMagicAction extends Action {

    private ArmSubsystem sub_ ;
    private String key_ ;

    private double lower_target_ ;
    private double lower_maxa_ ;
    private double lower_maxv_ ;
    private int lower_strength_ ;

    private boolean second_started_ ;

    private double upper_target_ ;
    private double upper_maxa_ ;
    private double upper_maxv_ ;
    private int upper_strength_ ;

    private MotorEncoderSubsystem first_subsystem_ ;
    private MotorEncoderMotionMagicAction first_action_ ;
    private boolean first_done_ ;

    private MotorEncoderSubsystem second_subsystem_ ;
    private MotorEncoderMotionMagicAction second_action_ ;
    private boolean second_done_ ;

    private double trigger_distance_ ;

    public ArmStaggeredGotoMagicAction(ArmSubsystem sub, String key) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;
        key_ = key ;

        lower_target_ = sub_.getSettingsValue(key + ":lower:target").getDouble();
        lower_maxa_ = sub_.getSettingsValue(key + ":lower:maxa").getDouble();
        lower_maxv_ = sub_.getSettingsValue(key + ":lower:maxv").getDouble();
        lower_strength_ = sub_.getSettingsValue(key + ":lower:strength").getInteger() ;

        upper_target_ = sub_.getSettingsValue(key + ":upper:target").getDouble();
        upper_maxa_ = sub_.getSettingsValue(key + ":upper:maxa").getDouble();
        upper_maxv_ = sub_.getSettingsValue(key + ":upper:maxv").getDouble();
        upper_strength_ = sub_.getSettingsValue(key + ":upper:strength").getInteger() ;

        if (sub_.isSettingDefined(key + ":upper:trigger")) {
            trigger_distance_ = sub_.getSettingsValue(key + ":upper:trigger").getDouble();
            second_subsystem_ = sub.getLowerSubsystem() ;
            second_action_ = new MotorEncoderMotionMagicAction(second_subsystem_, lower_target_, lower_maxa_, lower_maxv_, lower_strength_, HoldType.AtCurrentPosition);
            first_subsystem_ = sub.getUpperSubsystem() ;
            first_action_ = new MotorEncoderMotionMagicAction(first_subsystem_, upper_target_, upper_maxa_, upper_maxv_, upper_strength_, HoldType.AtCurrentPosition);

        }
        else if (sub_.isSettingDefined(key + ":lower:trigger")) {
            trigger_distance_ = sub_.getSettingsValue(key + ":lower:trigger").getDouble();
            first_subsystem_ = sub.getLowerSubsystem() ;
            first_action_ = new MotorEncoderMotionMagicAction(first_subsystem_, lower_target_, lower_maxa_, lower_maxv_, lower_strength_, HoldType.AtCurrentPosition);
            second_subsystem_ = sub.getUpperSubsystem() ;
            second_action_ = new MotorEncoderMotionMagicAction(second_subsystem_, upper_target_, upper_maxa_, upper_maxv_, upper_strength_, HoldType.AtCurrentPosition);

        }
        else {
            throw new Exception("there is no 'trigger' value specified for either the upper or lower arm") ;
        }
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        first_subsystem_.setDefaultAction(null);
        second_subsystem_.setDefaultAction(null);

        first_done_ = false ;
        second_done_ = false ;
        second_started_ = false ;

        first_subsystem_.setAction(first_action_, true) ;
    }
    
    @Override
    public void run() throws Exception {
        super.run() ;

        if (!second_started_ && first_action_.getDistance() > trigger_distance_) {
            second_started_ = true ;
            second_subsystem_.setAction(second_action_, true) ;
        }

        if (second_started_ && !second_done_ && second_action_.isComplete()) {
            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info) ;
            logger.add("ArmStaggeredGotoMagicAction: lower action complete @ ", sub_.getRobot().getTime()) ;
            logger.endMessage();
            second_done_ = true ;
        }

        if (!first_done_ && first_action_.isComplete()) {
            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info) ;
            logger.add("ArmStaggeredGotoMagicAction: upper action complete @ ", sub_.getRobot().getTime()) ;
            logger.endMessage();
            first_done_ = true ;
        }

        if (first_done_ && second_done_) {
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ArmStaggeredGotoMagicAction(" + key_ + ")";
    }
}
