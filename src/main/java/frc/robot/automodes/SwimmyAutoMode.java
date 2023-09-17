package frc.robot.automodes;

import java.util.function.Supplier;

import org.xero1425.base.actions.ConditionalAction;
import org.xero1425.base.actions.DelayAction;
import org.xero1425.base.actions.DispatchAction;
import org.xero1425.base.actions.ParallelAction;
import org.xero1425.base.actions.SequenceAction;
import org.xero1425.base.actions.ParallelAction.DonePolicy;
import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.AutoMode;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;

import frc.robot.subsystems.gpm.GPMCollectAction;
import frc.robot.subsystems.gpm.GPMPlaceAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.grabber.GrabberGrabLoadedGamepieceAction;
import frc.robot.subsystems.toplevel.AutoGamePieceAction;
import frc.robot.subsystems.toplevel.AutoGamePieceAction2;
import frc.robot.subsystems.toplevel.RobotOperation;
import frc.robot.subsystems.toplevel.Swimmy2023RobotSubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.GridTagPosition;
import frc.robot.subsystems.toplevel.RobotOperation.Location;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;

public class SwimmyAutoMode extends AutoMode {
    private GPMCollectAction collect1_ ;
    private GPMCollectAction collect2_ ;

    public SwimmyAutoMode(AutoController ctrl, String name) {
        super(ctrl, name) ;
    }

    protected void grabAndPlace(Location where, GamePiece what) throws Exception {
        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();

        //
        // Close the grabber around the game piece
        // 
        addSubActionPair(robot.getGPM().getGrabber(), new GrabberGrabLoadedGamepieceAction(robot.getGPM().getGrabber(), "start"), true);

        //
        // Place the game piece onto the placement location
        //
        addSubActionPair(robot.getGPM(), new GPMPlaceAction(robot.getGPM(), where, what, true, true), true);
    }

    protected void driveAndCollect2(String path, boolean setpose, double startcollect, double closecollect, GamePiece what) throws Exception {
        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();
        GPMSubsystem gpm = robot.getGPM() ;

        SwerveHolonomicPathFollower pathact = new SwerveHolonomicPathFollower(robot.getSwerve(), path, setpose, 0.2);
        collect1_ = new GPMCollectAction(robot.getGPM(), what, true) ;
        
        pathact.addDistanceBasedAction(startcollect, () -> { gpm.setAction(collect1_); }) ;
        pathact.addDistanceBasedAction(closecollect, () -> { collect1_.forcedClosed(); }) ;

        addAction(pathact);
    }

    protected void driveAndCollectAndPlace(String path, boolean setpose, double startcollect, double closecollect, double place, 
                                            GamePiece what, GridTagPosition tpos, Slot slot, Location loc) throws Exception {
        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();

        RobotOperation oper = new RobotOperation(RobotOperation.Action.Place, what, tpos, slot, loc);
        AutoGamePieceAction2 act = new AutoGamePieceAction2(robot, setpose, place, oper, path, 0.2) ;
        SwerveHolonomicPathFollower drive = act.getPathAction() ;

        collect2_ = new GPMCollectAction(robot.getGPM(), what, true) ;
        DispatchAction da = new DispatchAction(robot.getGPM(), collect2_, false);

        drive.addDistanceBasedAction(startcollect, () -> { robot.getGPM().setAction(da) ; }) ;
        drive.addDistanceBasedAction(closecollect, () -> { collect2_.forcedClosed();}) ;

        addAction(act) ;
    }

    protected void driveAndCollect(String path, boolean setpose, double collectdelay, double grabdelay, GamePiece what, double lambdist) throws Exception {
        Swimmy2023RobotSubsystem robot = (Swimmy2023RobotSubsystem)getAutoController().getRobot().getRobotSubsystem();

        SwerveHolonomicPathFollower pathact = new SwerveHolonomicPathFollower(robot.getSwerve(), path, setpose, 0.2);
        GPMCollectAction collect = new GPMCollectAction(robot.getGPM(), what, true) ;

        pathact.addDistanceBasedAction(lambdist, ()-> { collect.forcedClosed();});

        // pathact.setLambda(()-> { collect.forcedClosed();}, lambdist);

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
