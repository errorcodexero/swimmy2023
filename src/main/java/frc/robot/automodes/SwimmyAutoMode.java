package frc.robot.automodes;

import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.gpm.GPMPlaceAction;
import frc.robot.subsystems.gpm.GPMStartWithGPAction;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class SwimmyAutoMode extends AutoMode {
    AutoController ctrl_;
    Swimmy2023RobotSubsystem robot_;
    public SwimmyAutoMode(AutoController ctrl, String name) {
        super(ctrl, name) ;
        ctrl_ = ctrl;
        robot_ = (Swimmy2023RobotSubsystem)ctrl_.getRobot().getRobotSubsystem();
    }

    protected void grabAndPlace(Location location, GamePiece piece) throws InvalidActionRequest, BadParameterTypeException, MissingParameterException {
        addSubActionPair(robot_.getGPM(), new GPMStartWithGPAction(robot_.getGPM()), true);
        addSubActionPair(robot_.getGPM(), new GPMPlaceAction(robot_.getGPM(), location, piece, true), true);
    }

    protected void drivePath(String name, boolean setpose, double endtime) throws InvalidActionRequest, BadParameterTypeException, MissingParameterException {
        addSubActionPair(robot_.getSwerve(), new SwerveHolonomicPathFollower(robot_.getSwerve(), name, setpose, endtime), true);

    }
}
