package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.InvalidActionRequest;
import org.xero1425.base.actions.ParallelAction;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.actions.ParallelAction.DonePolicy;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import frc.robot.subsystems.gpm.GPMCollectAction;
import frc.robot.subsystems.gpm.GPMPlaceAction;
import frc.robot.subsystems.grabber.GrabberGrabLoadedGamepieceAction;
import frc.robot.subsystems.toplevel.AutoGamePieceAction;
import frc.robot.subsystems.toplevel.RobotOperation;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.Action;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.GridTagPosition;
import frc.robot.subsystems.toplevel.RobotOperation.Location;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;

public class SwimmyAutoMode extends AutoMode {
    public SwimmyAutoMode(AutoController ctrl, String name) {
        super(ctrl, name) ;
    }

    protected void grabAndPlace(Location where, GamePiece what) throws InvalidActionRequest, BadParameterTypeException, MissingParameterException {
        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();

        //
        // Close the grabber around the game piece
        // 
        addSubActionPair(robot.getGPM().getGrabber(), new GrabberGrabLoadedGamepieceAction(robot.getGPM().getGrabber()), true);

        //
        // Place the game piece onto the placement location
        //
        addSubActionPair(robot.getGPM(), new GPMPlaceAction(robot.getGPM(), where, what, true), true);
    }

    protected void driveAndCollect(String path, boolean setpose, double collectdelay, double grabdelay, GamePiece what) throws Exception {
        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();

        ParallelAction action = new ParallelAction(getMessageLogger(), DonePolicy.All) ;

        //
        // Drive path 1, across the platform to find another game piece.
        //
        SwerveHolonomicPathFollower act = new SwerveHolonomicPathFollower(robot.getSwerve(), path, setpose, 1.0);
        action.addSubActionPair(robot.getSwerve(), act , true);

        //
        // In parallel with path 1 above, delay a fixed amount of time and then enter a ground collect operation
        //
        SequenceAction delaycollect = new SequenceAction(getMessageLogger());
        delaycollect.addAction(new DelayAction(getAutoController().getRobot(), collectdelay));
        delaycollect.addSubActionPair(robot.getGPM(), new GPMCollectAction(robot.getGPM(), what, true), true);
        action.addAction(delaycollect);

        addAction(action);
    }
    
    protected void driveAndPlace(String path, GridTagPosition tpos, Slot slot, Location loc, GamePiece what) throws InvalidActionRequest, BadParameterTypeException, MissingParameterException {
        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();

        RobotOperation oper = new RobotOperation(Action.Place, what, tpos, slot, loc);
        addSubActionPair(robot, new AutoGamePieceAction(robot, oper, path, 1.0), true);
    }

    protected void drivePath(String path, boolean setpose) throws Exception {
        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();

        //
        // Drive path 1, across the platform to find another game piece.
        //
        SwerveHolonomicPathFollower act = new SwerveHolonomicPathFollower(robot.getSwerve(), path, setpose, 1.0);
        addSubActionPair(robot.getSwerve(), act , true);
    }    
}
