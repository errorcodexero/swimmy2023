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
import frc.robot.subsystems.swerve.SwerveDriveBalancePlatform;
import frc.robot.subsystems.swerve.SwerveDriveBalancePlatform.XYDirection;
import frc.robot.subsystems.toplevel.RobotOperation;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class SwimmyAutoModeCenter1 extends SwimmyAutoMode  {
    public SwimmyAutoModeCenter1(AutoController ctrl, Location where, GamePiece what, boolean placetwo) throws Exception {
        super(ctrl, "Center-1") ;

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
        // Drive path 1, across the platform to find another game piece.
        //
        action.addSubActionPair(robot.getSwerve(), new SwerveHolonomicPathFollower(robot.getSwerve(), "Center1-Path1", true, 1.0) , true);

        //
        // In parallel with path 1 above, delay a fixed amount of time and then enter a ground collect operation
        //
        SequenceAction delaycollect = new SequenceAction(getMessageLogger());
        delaycollect.addAction(new DelayAction(getAutoController().getRobot(), "automodes:center1:before-collect-delay"));
        double grabdelay = robot.getRobot().getSettingsSupplier().get("automodes:center:collect-delay").getDouble();
        delaycollect.addSubActionPair(robot.getGPM(), new GPMCollectAction(robot.getGPM(), RobotOperation.GamePiece.Cone, true, grabdelay), true);
        action.addAction(delaycollect);

        addAction(action);

        //
        // Drive to the center of the platform rotating 90 degrees
        //
        action.addSubActionPair(robot.getSwerve(), new SwerveHolonomicPathFollower(robot.getSwerve(), "Center1-Path2", false, 1.0) , true);

        //
        // Delay for 1 second to be sure the platform has settled
        //
        addAction(new DelayAction(getAutoController().getRobot(), 1.0));

        //
        // Now run the auto balance operation (with the robot sideways)
        //
        action.addSubActionPair(robot.getSwerve(), new SwerveDriveBalancePlatform(robot.getSwerve(), XYDirection.YDirection), true);
    }
}