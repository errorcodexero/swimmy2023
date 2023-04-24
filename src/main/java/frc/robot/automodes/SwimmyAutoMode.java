package frc.robot.automodes;

import java.util.function.Supplier;

import org.xero1425.base.actions.ConditionalAction;
import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.DispatchAction;
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
        addSubActionPair(robot.getGPM(), new GPMPlaceAction(robot.getGPM(), where, what, true, true), true);
    }

    protected void driveAndCollect(String path, boolean setpose, double collectdelay, double grabdelay, GamePiece what, double lambdist) throws Exception {
        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();

        SwerveHolonomicPathFollower pathact = new SwerveHolonomicPathFollower(robot.getSwerve(), path, setpose, 0.2);
        GPMCollectAction collect = new GPMCollectAction(robot.getGPM(), what, true) ;

        pathact.setLambda(()-> { collect.forcedClosed();}, lambdist);

        ParallelAction action = new ParallelAction(getMessageLogger(), DonePolicy.All) ;

        //
        // Drive path 1, across the platform to find another game piece.
        //
        SequenceAction seq = new SequenceAction(robot.getRobot().getMessageLogger()) ;
        seq.addSubActionPair(robot.getSwerve(), pathact , true);

        action.addAction(seq);

        //
        // In parallel with path 1 above, delay a fixed amount of time and then enter a ground collect operation
        //
        SequenceAction delaycollect = new SequenceAction(getMessageLogger());
        delaycollect.addAction(new DelayAction(getAutoController().getRobot(), collectdelay));
        delaycollect.addSubActionPair(robot.getGPM(), collect , true);
        action.addAction(delaycollect);

        addAction(action);
    }
    
    protected void driveAndPlace(String path, GridTagPosition tpos, Slot slot, Location loc, GamePiece what) throws Exception {
        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();

        RobotOperation oper = new RobotOperation(RobotOperation.Action.Place, what, tpos, slot, loc);

        Supplier<Boolean> hasGP = () -> { return robot.getGPM().getGrabber().getSensor() ; } ;
        DispatchAction trueact = new DispatchAction(robot, new AutoGamePieceAction(robot, oper, path, 1.0), true);
        GPMCollectAction falseact = new GPMCollectAction(robot.getGPM(), GamePiece.Cone, true) ;
        ConditionalAction cond = new ConditionalAction(robot.getRobot().getMessageLogger(), hasGP, trueact, falseact) ;
        addAction(cond);
    }

    protected void drivePath(String path, boolean setpose) throws Exception {
        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();

        SwerveHolonomicPathFollower act = new SwerveHolonomicPathFollower(robot.getSwerve(), path, setpose, 1.0);
        addSubActionPair(robot.getSwerve(), act , true);
    }    
}
