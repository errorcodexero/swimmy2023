package frc.robot.subsystems.toplevel;

import org.xero1425.base.actions.Action;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class AutoGamePieceAction extends Action {
    private RobotOperation oper_ ;
    private Swimmy2023RobotSubsystem sub_ ;

    public AutoGamePieceAction(Swimmy2023RobotSubsystem sub, RobotOperation oper) {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        oper_ = oper;
    }

    @Override
    public void start() {
        if (!sub_.setOperation(oper_)) {
            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Error);
            logger.add("action AutoGamePlaceAction, operation " + oper_.toString() + ", was rejected by the subsystem");
            logger.endMessage();
            setDone() ;
        }
    }

    @Override
    public void run() {
        if (sub_.isOperationComplete()) {
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "AutoGamePieceAction " + oper_.toString() ;
    }
}
