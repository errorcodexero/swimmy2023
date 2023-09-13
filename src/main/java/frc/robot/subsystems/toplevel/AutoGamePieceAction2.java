package frc.robot.subsystems.toplevel;

import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.base.actions.Action;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

public class AutoGamePieceAction2 extends Action {
    private double dist_ ;
    private RobotOperation oper_ ;
    private boolean set_oper_ ;
    private Swimmy2023RobotSubsystem sub_ ;
    private String pathname_ ;
    private SwerveHolonomicPathFollower drive_action_ ;

    public AutoGamePieceAction2(Swimmy2023RobotSubsystem sub, boolean setpose, double dist, RobotOperation oper, String pathname, double delay) throws BadParameterTypeException, MissingParameterException {
        super(sub.getRobot().getMessageLogger());
        dist_ = dist ;
        sub_ = sub;
        oper_ = oper;

        SwerveBaseSubsystem swerve = (SwerveBaseSubsystem)sub_.getRobot().getRobotSubsystem().getDB();
        drive_action_ = new SwerveHolonomicPathFollower(swerve, pathname, setpose, delay) ;
        drive_action_.disableVision(false);
        pathname_ = pathname;
    }

    public SwerveHolonomicPathFollower getPathAction() {
        return drive_action_ ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        set_oper_ = false ;

        if (dist_ == 0.0) {
            if (!sub_.setOperation(oper_)) {
                MessageLogger logger = sub_.getRobot().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("action AutoGamePlaceAction, operation " + oper_.toString() + ", was rejected by the subsystem");
                logger.endMessage();
                setDone() ;
                return ;
            }

            set_oper_ = true ;
        }

        if (pathname_ != null) {
            SwerveBaseSubsystem swerve = (SwerveBaseSubsystem)sub_.getRobot().getRobotSubsystem().getDB();
            swerve.setAction(drive_action_, true);
        }
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (!set_oper_ && drive_action_.getDistance() > dist_) {
            if (!sub_.setOperation(oper_)) {
                MessageLogger logger = sub_.getRobot().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("action AutoGamePlaceAction, operation " + oper_.toString() + ", was rejected by the subsystem");
                logger.endMessage();
                setDone() ;
                return ;
            }

            set_oper_ = true ;            
        }

        if (set_oper_ && sub_.isOperationComplete() && (drive_action_.isDone() || sub_.getSwerve().getAction() != drive_action_)) {
            MessageLogger logger = sub_.getRobot().getMessageLogger();
            logger.startMessage(MessageType.Error);
            logger.add("Running AutoGamePieceAction") ;
            logger.add("subcomplete", sub_.isOperationComplete());
            logger.add("path", drive_action_.isDone());
            logger.endMessage(); 
            setDone();
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "AutoGamePieceAction2 " + oper_.toString() ;
    }
}
