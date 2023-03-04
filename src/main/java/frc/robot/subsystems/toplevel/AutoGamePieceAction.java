package frc.robot.subsystems.toplevel;

import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

public class AutoGamePieceAction extends Action {
    private RobotOperation oper_ ;
    private Swimmy2023RobotSubsystem sub_ ;
    private String pathname_ ;
    private double delay_ ;
    private SwerveHolonomicPathFollower drive_action_ ;

    public AutoGamePieceAction(Swimmy2023RobotSubsystem sub, RobotOperation oper) {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        oper_ = oper;
    }

    public AutoGamePieceAction(Swimmy2023RobotSubsystem sub, RobotOperation oper, String pathname, double delay) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        oper_ = oper;

        SwerveBaseSubsystem swerve = (SwerveBaseSubsystem)sub_.getRobot().getRobotSubsystem().getDB();
        drive_action_ = new SwerveHolonomicPathFollower(swerve, pathname, false, delay_) ;
        drive_action_.disableVision(false);
        pathname_ = pathname;
        delay_ = delay;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        if (!sub_.setOperation(oper_)) {
            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Error);
            logger.add("action AutoGamePlaceAction, operation " + oper_.toString() + ", was rejected by the subsystem");
            logger.endMessage();
            setDone() ;
        }

        if (pathname_ != null) {
            SwerveBaseSubsystem swerve = (SwerveBaseSubsystem)sub_.getRobot().getRobotSubsystem().getDB();
            swerve.setAction(drive_action_, true);
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (pathname_ == null) {
            if (sub_.isOperationComplete()) {
                setDone();
            }
        }
        else {
            if (sub_.isOperationComplete() && drive_action_.isDone()) {
                setDone();
            }
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "AutoGamePieceAction " + oper_.toString() ;
    }
}
