package frc.robot.subsystems.arm;

import org.xero1425.base.actions.Action;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderMotionMagicActon;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderMotionMagicActon.HoldType;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class ArmStaggeredGotoMagicAction extends Action {

    private ArmSubsystem sub_ ;
    private String key_ ;

    private double l1_target_ ;
    private double l1_maxa_ ;
    private double l1_maxv_ ;
    private int l1_strength_ ;
    private double l1_delay_ ;
    private MotorEncoderMotionMagicActon l1_action_ ;
    private boolean l1_done_ ;

    private double l2_target_ ;
    private double l2_maxa_ ;
    private double l2_maxv_ ;
    private int l2_strength_ ;
    private double l2_delay_ ;
    private boolean l2_done_ ;
    private MotorEncoderMotionMagicActon l2_action_ ;

    public ArmStaggeredGotoMagicAction(ArmSubsystem sub, String key) throws Exception {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;
        key_ = key ;

        l1_target_ = sub_.getSettingsValue(key + ":lower:target").getDouble();
        l1_maxa_ = sub_.getSettingsValue(key + ":lower:config:maxa").getDouble();
        l1_maxv_ = sub_.getSettingsValue(key + ":lower:config:maxv").getDouble();
        l1_delay_ = sub_.getSettingsValue(key + ":lower:delay").getDouble();

        if (sub_.isSettingDefined(key + ":lower:strength")) {
            l1_strength_ = sub_.getSettingsValue(key + ":lower:strength").getInteger() ;
        } else {
            l1_strength_ = 8 ;
        }

        l2_target_ = sub_.getSettingsValue(key + ":upper:target").getDouble();
        l2_maxa_ = sub_.getSettingsValue(key + ":upper:config:maxa").getDouble();
        l2_maxa_ = sub_.getSettingsValue(key + ":upper:config:maxv").getDouble();
        l1_delay_ = sub_.getSettingsValue(key + ":upper:delay").getDouble();

        if (sub_.isSettingDefined(key + ":upper:strength")) {
            l2_strength_ = sub_.getSettingsValue(key + ":upper:strength").getInteger() ;
        } else {
            l2_strength_ = 8 ;
        }

        l1_action_ = new MotorEncoderMotionMagicActon(sub.getLowerSubsystem(), l1_delay_, l1_target_, l1_maxa_, l1_maxv_, l1_strength_, HoldType.AtCurrentPosition);
        l2_action_ = new MotorEncoderMotionMagicActon(sub.getUpperSubsystem(), l2_delay_, l2_target_, l2_maxa_, l2_maxv_, l2_strength_, HoldType.AtCurrentPosition);
    }

    @Override
    public void start() throws Exception {
        l1_done_ = false ;
        l2_done_ = false ;

        sub_.getLowerSubsystem().setAction(l1_action_);
        sub_.getUpperSubsystem().setAction(l2_action_);
    }
    
    @Override
    public void run() throws Exception {
        if (!l1_done_ && l1_action_.isComplete()) {
            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info) ;
            logger.add("ArmStaggeredGotoMagicAction: L1 action complete @ ", sub_.getRobot().getTime()) ;
            logger.endMessage();
            l1_done_ = true ;
        }

        if (!l2_done_ && l2_action_.isComplete()) {
            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Info) ;
            logger.add("ArmStaggeredGotoMagicAction: L2 action complete @ ", sub_.getRobot().getTime()) ;
            logger.endMessage();
            l2_done_ = true ;
        }

        if (l1_action_.isComplete() &&  l2_action_.isComplete()) {
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "ArmStaggeredGotoAction(" + key_ + ")";
    }
}
