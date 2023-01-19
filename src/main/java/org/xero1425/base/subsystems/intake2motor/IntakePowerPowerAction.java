package org.xero1425.base.subsystems.intake2motor;

import org.xero1425.base.subsystems.motorsubsystem.MotorPowerAction;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

//
// Note, this action will be "done" when the motor encoder action is complete
// and the intake is in the requested position.  The spinner will continue with the
// power assigned and must be explicitly stopped if necessary.
//
// For an intake "off" style action, if the spinner is off while raising the intake
// arm, then nothing is required.  However if you are running the spinner in reverse
// while raising the intake arm, then when this action is done
//
public class IntakePowerPowerAction extends MotorPowerAction {
    private Intake2MotorSubsystem sub_ ;
    private double collect_power_ ;

    public IntakePowerPowerAction(Intake2MotorSubsystem sub, String updownpower, String power)  throws Exception {
        super(sub, updownpower) ;

        sub_ = sub ;
        collect_power_ = sub.getSettingsValue(power).getDouble() ;
    }

    public IntakePowerPowerAction(Intake2MotorSubsystem sub, double updownpower, double power)  throws Exception {
        super(sub, updownpower) ;

        sub_ = sub ;
        collect_power_ = power ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
        logger.add("start") ;
        logger.add("isdone", isDone()) ;
        logger.add("power", collect_power_) ;
        logger.endMessage();

        sub_.setSpinnerPower(collect_power_);
    }

    @Override
    public void cancel() {
        super.cancel() ;

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
        logger.add("cancel") ;
        logger.add("isdone", isDone()) ;
        logger.add("power", collect_power_) ;
        logger.endMessage();

        try {
            sub_.setSpinnerPower(0.0) ;
        }
        catch(Exception ex) {
        }
    }
}
