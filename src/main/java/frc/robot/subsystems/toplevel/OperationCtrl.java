package frc.robot.subsystems.toplevel;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;

public abstract class OperationCtrl {
    private Swimmy2023RobotSubsystem sub_ ;
    private RobotOperation oper_ ;
    private boolean done_ ;

    public OperationCtrl(Swimmy2023RobotSubsystem sub, RobotOperation oper) {
        oper_ = oper ;
        sub_ = sub ;
    }

    public void updateGamePiece(GamePiece gp) {
    }

    public RobotOperation getOper() {
        return oper_ ;
    }

    public Swimmy2023RobotSubsystem getRobotSubsystem() {
        return sub_ ;
    }

    protected FieldLocationData getFieldData() {
        return sub_.getFieldData() ;
    }

    public void start() throws BadParameterTypeException, MissingParameterException {
        done_ = false ;
    }

    public abstract void run() throws BadParameterTypeException, MissingParameterException ;
    public abstract void abort() throws BadParameterTypeException, MissingParameterException ;

    public boolean isDone() {
        return done_ ;
    }

    protected void setDone() {
        done_ = true ;
    }
}
