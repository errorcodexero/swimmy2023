package org.xero1425.base.subsystems.intake2motor;

import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderGotoAction;
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
public class IntakePositionPowerAction extends MotorEncoderGotoAction {
    private Intake2MotorSubsystem sub_ ;
    private double collect_power_ ;
    private boolean stop_spinner_when_done_ ;

    public IntakePositionPowerAction(Intake2MotorSubsystem sub, String pos, String power, boolean stop_when_done, boolean addhold)  throws Exception {
        super(sub, pos, addhold) ;

        sub_ = sub ;
        collect_power_ = sub.getSettingsValue(power).getDouble() ;
        stop_spinner_when_done_ = stop_when_done ;
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
    public void run() throws Exception {
        super.run() ;

        MessageLogger logger = getSubsystem().getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getSubsystem().getLoggerID()) ;
        logger.add("run") ;
        logger.add("isdone", isDone()) ;
        logger.add("power", collect_power_) ;
        logger.endMessage();

        if (isDone() && stop_spinner_when_done_) {
            sub_.setSpinnerPower(0.0) ;
        }
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
