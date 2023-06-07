package frc.robot.subsystems.arm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderMotionMagicActon;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderMotionMagicActon.HoldType;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class ArmStaggeredGotoMagicAction extends Action {

    private ArmSubsystem sub_ ;
    private String key_ ;

    private double lower_target_ ;
    private double lower_maxa_ ;
    private double lower_maxv_ ;
    private int lower_strength_ ;
    private MotorEncoderMotionMagicActon lower_action_ ;
    private boolean lower_done_ ;
    private boolean lower_started_ ;

    private double upper_target_ ;
    private double upper_maxa_ ;
    private double upper_maxv_ ;
    private int upper_strength_ ;
    private boolean upper_done_ ;
    private MotorEncoderMotionMagicActon upper_action_ ;

    private double trigger_distance_ ;

    public ArmStaggeredGotoMagicAction(ArmSubsystem sub, String key) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;
        key_ = key ;

        lower_target_ = sub_.getSettingsValue(key + ":lower:target").getDouble();
        lower_maxa_ = sub_.getSettingsValue(key + ":lower:config:maxa").getDouble();
        lower_maxv_ = sub_.getSettingsValue(key + ":lower:config:maxv").getDouble();

        if (sub_.isSettingDefined(key + ":lower:strength")) {
            lower_strength_ = sub_.getSettingsValue(key + ":lower:strength").getInteger() ;
        } else {
            lower_strength_ = 8 ;
        }

        upper_target_ = sub_.getSettingsValue(key + ":upper:target").getDouble();
        upper_maxa_ = sub_.getSettingsValue(key + ":upper:config:maxa").getDouble();
        upper_maxa_ = sub_.getSettingsValue(key + ":upper:config:maxv").getDouble();

        if (sub_.isSettingDefined(key + ":upper:strength")) {
            upper_strength_ = sub_.getSettingsValue(key + ":upper:strength").getInteger() ;
        } else {
            upper_strength_ = 8 ;
        }

        trigger_distance_ = sub_.getSettingsValue(key + ":upper:trigger").getDouble();

        lower_action_ = new MotorEncoderMotionMagicActon(sub.getLowerSubsystem(), lower_target_, lower_maxa_, lower_maxv_, lower_strength_, HoldType.AtCurrentPosition);
        upper_action_ = new MotorEncoderMotionMagicActon(sub.getUpperSubsystem(), upper_target_, upper_maxa_, upper_maxv_, upper_strength_, HoldType.AtCurrentPosition);
    }

    @Override
    public void start() throws Exception {
        lower_done_ = false ;
        lower_started_ = false ;
        upper_done_ = false ;

        sub_.getUpperSubsystem().setAction(upper_action_);
    }
    
    @Override
    public void run() throws Exception {

        if (!lower_started_ && upper_action_.getDistance() > trigger_distance_) {
            lower_started_ = true ;
            sub_.getLowerSubsystem().setAction(lower_action_);
        }

        if (lower_started_ && !lower_done_ && lower_action_.isComplete()) {
            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info) ;
            logger.add("ArmStaggeredGotoMagicAction: lower action complete @ ", sub_.getRobot().getTime()) ;
            logger.endMessage();
            lower_done_ = true ;
        }

        if (!upper_done_ && upper_action_.isComplete()) {
            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info) ;
            logger.add("ArmStaggeredGotoMagicAction: upper action complete @ ", sub_.getRobot().getTime()) ;
            logger.endMessage();
            upper_done_ = true ;
        }

        if (lower_done_ && upper_done_) {
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ArmStaggeredGotoAction(" + key_ + ")";
    }
}
