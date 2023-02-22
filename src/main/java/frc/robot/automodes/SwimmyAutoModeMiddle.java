package frc.robot.automodes;

import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.ParallelAction;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.actions.ParallelAction.DonePolicy;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;

import frc.robot.subsystems.gpm.GPMCollectAction;
import frc.robot.subsystems.gpm.GPMPlaceAction;
import frc.robot.subsystems.grabber.GrabberGrabGampieceAction;
import frc.robot.subsystems.toplevel.AutoGamePieceAction;
import frc.robot.subsystems.toplevel.RobotOperation;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.Action;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.GridTagPosition;
import frc.robot.subsystems.toplevel.RobotOperation.Location;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;

public class SwimmyAutoModeMiddle extends SwimmyAutoMode  {
    public SwimmyAutoModeMiddle(AutoController ctrl, Location where, GamePiece what) throws Exception {
        super(ctrl, "Middle") ;

        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();

        //
        // Close the grabber around the game piece
        // 
        addSubActionPair(robot.getGPM(), new GrabberGrabGampieceAction(robot.getGPM().getGrabber(), RobotOperation.GamePiece.Cone), true);

        //
        // Place the game piece onto the placement location
        //
        addSubActionPair(robot.getGPM(), new GPMPlaceAction(robot.getGPM(), where, what, true), true);
        ParallelAction action = new ParallelAction(getMessageLogger(), DonePolicy.All) ;

        //
        // Drive path 1, across the platform to find another game piece.  This is in parallel with getting the
        // ARM and grabber ready to collect
        //
        action.addSubActionPair(robot.getSwerve(), new SwerveHolonomicPathFollower(robot.getSwerve(), "CenterMode-Path1", true, 1.0) , true);

        SequenceAction delaycollect = new SequenceAction(getMessageLogger());
        delaycollect.addAction(new DelayAction(getAutoController().getRobot(), "automodes:center:before-collect-delay"));

        double grabdelay = robot.getRobot().getSettingsSupplier().get("automodes:center:collect-delay").getDouble();
        delaycollect.addSubActionPair(robot.getGPM(), new GPMCollectAction(robot.getGPM(), true, grabdelay), true);

        action.addAction(delaycollect);
        addAction(action);

        //
        // Drive path 2 and place the piece
        //
        RobotOperation oper = new RobotOperation(Action.Place, GamePiece.Cone, GridTagPosition.Middle, Slot.Right, Location.Middle);
        AutoGamePieceAction act = new AutoGamePieceAction(robot, oper, "CenterMode-Path2", 1.0);
        addSubActionPair(robot, act, true);

        //
        // Drive path 3 to the center of the platform
        //
        action.addSubActionPair(robot.getSwerve(), new SwerveHolonomicPathFollower(robot.getSwerve(), "CenterMode-Path3", false, 1.0) , true);
    }
}
