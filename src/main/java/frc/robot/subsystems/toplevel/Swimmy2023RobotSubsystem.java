package frc.robot.subsystems.toplevel;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveToPoseAction;
import org.xero1425.base.subsystems.swerve.sdsswerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.SwimmyRobot2023;
import frc.robot.subsystems.gpm.GPMCollectAction;
import frc.robot.subsystems.gpm.GPMPlaceAction;
import frc.robot.subsystems.gpm.GPMStowAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.oi.Swimmy2023OISubsystem;
import frc.robot.subsystems.toplevel.RobotOperation.Action;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;


public class Swimmy2023RobotSubsystem extends RobotSubsystem {

    enum State {
        Idle,

        //
        // Common to all automatic operations
        //
        LookingForTag,
        DrivingToLocation,

        //
        // Manual collect operation
        //
        ManualWaitForCollectDone,

        //
        // Manual place operation
        //
        ManualWaitForPlaceDone
    } ;

    //
    // The subsystems
    //
    private GPMSubsystem gpm_;
    private SDSSwerveDriveSubsystem db_;
    private Swimmy2023OISubsystem oi_;
    private LimeLightSubsystem limelight_;

    //
    // Actions we assign to the various subsystems
    //
    private SwerveDriveToPoseAction drive_to_;
    private GPMCollectAction gpm_shelf_collect_action_ ;
    private GPMPlaceAction gpm_place_action_ ;
    private GPMStowAction gpm_stow_action_ ;

    //
    // The current operation
    //
    private FieldLocationData locdata_ ;
    private RobotOperation operation_ ;
    private State state_ ;
    private Pose2d target_pose_ ;
    
    public Swimmy2023RobotSubsystem(XeroRobot robot) throws Exception {
        super(robot, "Swimmy2023RobotSubsystem") ;

        SwimmyRobot2023 swimmy = (SwimmyRobot2023)robot ;
        locdata_ = swimmy.getFieldData() ;

        db_ = new SDSSwerveDriveSubsystem(this, "swerve" );
        addChild(db_);

        oi_ = new Swimmy2023OISubsystem(this, db_);
        addChild(oi_);

        limelight_ = new LimeLightSubsystem(this, "limelight");
        addChild(limelight_);
        
        gpm_ = new GPMSubsystem(this);
        addChild(gpm_);     

        db_.setVision(limelight_);
        
        gpm_shelf_collect_action_ = new GPMCollectAction(gpm_);
        gpm_stow_action_ = new GPMStowAction(gpm_);

        operation_ = null;
        state_ = State.Idle;
    }

    public GPMSubsystem getGPM() {
        return gpm_ ;
    }

    public SwerveBaseSubsystem getSwerve() {
        return db_ ;
    }

    public Swimmy2023OISubsystem getOI() {
        return oi_ ;
    }

    public LimeLightSubsystem getLimeLight() {
        return limelight_ ;
    }

    private boolean isOperationValid(RobotOperation oper) {
        MessageLogger logger = getRobot().getMessageLogger();
        boolean ret = true ;

        if (oper.getAction() == Action.Collect) {
            //
            // Rules for collect
            //
            if (oper.getSlot() == Slot.Middle) {
                logger.startMessage(MessageType.Error);
                logger.add("Swimmy2023RobotSubsystem: invalid operation assigned - collect operation must be slot Left or Right");
                logger.endMessage();
                ret = false ;
            }
        }
        else {
            //
            // Rules for placement
            // 
        }

        return ret ;
    }

    public boolean setOperation(RobotOperation oper) {
        if (operation_ != null) {
            //
            // If another operation is set, we cannot override it.  The gunner must press
            // abort to stop the current operation before assigning a new operation.
            //
            return false ;
        }

        if (!isOperationValid(oper)) {
            return false ;
        }

        MessageLogger logger = getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        logger.add("Swimmy2023RobotSubsystem: starting: " + oper.toString()) ;
        logger.endMessage();

        operation_ = oper ;
        return true ;
    }

    public void abort() {
        switch(state_) {
            case Idle:
                break ;

            case LookingForTag:
                break ;

            case DrivingToLocation:
                oi_.enableGamepad() ;
                oi_.getGamePad().rumble(1.0, 2.0);
                drive_to_.cancel() ;
                break ;
                
            case ManualWaitForCollectDone:
                getGPM().setAction(gpm_stow_action_);
                break ;

            case ManualWaitForPlaceDone:
                getGPM().setAction(gpm_stow_action_);
                break;
        }

        state_ = State.Idle;
        operation_ = null;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        Gamepad gp = oi_.getGamePad() ;
        if (gp != null && gp.isEnabled() == false && gp.isXPressed() && gp.isAPressed()) {
            abort() ;
            gp.enable();
        }

        if (getAction() != null || operation_ == null) {
            return ;
        }

        //
        // What are we doing with the current operation
        //
        State orig = state_ ;
        switch(state_) {
            case Idle:
                stateIdle() ;
                break;

            case LookingForTag:
                stateLookingForTag() ;
                break;

            case DrivingToLocation:
                stateDriveToLocation();
                break ;

            case ManualWaitForCollectDone:
                stateManualWaitForCollectDone();
                break;

            case ManualWaitForPlaceDone:
                stateManualWaitForPlaceDone();
                break;
        }

        if (state_ != orig) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getLoggerID());
            logger.add("State Changes: " + orig.toString() + " -> " + state_.toString());
            logger.endMessage();
        }
    }

    private int getTargetTag() {
        int tag = -1 ;

        if (operation_.getAction() == Action.Collect) {
            tag = locdata_.getLoadingStationTag() ;
        }
        else {
            tag = locdata_.getGridTag(operation_.getAprilTag());
        }

        return tag ;
    }

    private void stateIdle() throws MissingParameterException, BadParameterTypeException {
        if (operation_.getManual()) {
            if (operation_.getAction() == RobotOperation.Action.Collect) {
                getGPM().setAction(gpm_shelf_collect_action_, true) ;
            }
            else {
                gpm_place_action_ = new GPMPlaceAction(gpm_, operation_.getLocation(), operation_.getGamePiece());
                getGPM().setAction(gpm_place_action_, true);
            }

            state_ = State.ManualWaitForCollectDone;
        }
        else {
            state_ = State.LookingForTag ;
        }
    }

    private void stateDriveToLocation() {
        if (operation_.getAction() == Action.Collect) {
            //
            // We are collecting, the drive to location and the shelf collect action
            // are running in parallel.  The arm should be deployed and when the drive base
            // drives to the game piece, we should detect it and finish the collect.  This
            // finishes the collect operation
            //
            if (gpm_shelf_collect_action_.isDone()) {
                oi_.enableGamepad() ;
                oi_.getGamePad().rumble(1.0, 2.0);
                state_ = State.Idle;
                operation_ = null ;
            }
        }
        else {
            //
            // We are placing.
            //
            
        }
    }

    private void stateLookingForTag() throws BadParameterTypeException, MissingParameterException {
        int tag = getTargetTag() ;

        MessageLogger logger = getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Debug, getLoggerID());
        logger.add("looking for april tag");
        logger.endMessage();

        if (limelight_.hasAprilTag(tag)) {
            oi_.disableGamepad() ;

            if (operation_.getAction() == RobotOperation.Action.Collect) {
                target_pose_ = locdata_.getGridPose(tag, operation_.getSlot());
            }
            else {
                target_pose_ = locdata_.getLoadingStationPose(operation_.getSlot());
            }

            drive_to_ = new SwerveDriveToPoseAction(db_, target_pose_);
            db_.setAction(drive_to_);
            gpm_.setAction(gpm_shelf_collect_action_, true);

            state_ = State.DrivingToLocation;
        }
    }

    private void stateManualWaitForCollectDone() {
        if (gpm_stow_action_.isDone()) {
            operation_ = null;
            state_ = State.Idle ;
        }
    }

    private void stateManualWaitForPlaceDone() {
        if (gpm_place_action_.isDone()) {
            operation_ = null;
            gpm_place_action_ = null;
            state_ = State.Idle ;
        }
    }
}