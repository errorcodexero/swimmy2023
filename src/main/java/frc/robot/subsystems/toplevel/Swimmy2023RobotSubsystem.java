package frc.robot.subsystems.toplevel;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.RobotSubsystem;
import org.xero1425.base.subsystems.oi.Gamepad;
import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveToPoseAction;
import org.xero1425.base.subsystems.swerve.sdsswerve.SDSSwerveDriveSubsystem;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.gpm.GPMCollectAction;
import frc.robot.subsystems.gpm.GPMSubsystem;
import frc.robot.subsystems.oi.Swimmy2023OISubsystem;


public class Swimmy2023RobotSubsystem extends RobotSubsystem {

    enum State {
        Idle,
        LookingForTag,
        DrivingToLocation
    } ;

    //
    // The subsystems
    //
    private GPMSubsystem gpm_;
    private SDSSwerveDriveSubsystem db_;
    private Swimmy2023OISubsystem oi_;
    private LimeLightSubsystem limelight_;

    //
    // Location data for the game
    // 
    private LocationData locdata_ ;

    //
    // Actions we assign to the various subsystems
    //
    private SwerveDriveToPoseAction drive_to_;
    private GPMCollectAction shelf_collect_action_ ;

    //
    // The current operation
    //
    private RobotOperation operation_ ;
    private State state_ ;
    private Pose2d target_pose_ ;
    
    public Swimmy2023RobotSubsystem(XeroRobot robot) throws Exception {
        super(robot, "Swimmy2023RobotSubsystem") ;

        db_ = new SDSSwerveDriveSubsystem(this, "swerve" );
        addChild(db_);

        oi_ = new Swimmy2023OISubsystem(this, db_);
        addChild(oi_);

        limelight_ = new LimeLightSubsystem(this, "limelight");
        addChild(limelight_);
        
        gpm_ = new GPMSubsystem(this);
        addChild(gpm_);     
        
        locdata_ = new LocationData();

        shelf_collect_action_ = new GPMCollectAction(gpm_);

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

    public boolean setOperation(RobotOperation oper) {
        boolean ret = false ;

        if (operation_ == null) {
            ret = true ;
            operation_ = oper ;
            state_ = State.LookingForTag;
        } 

        return ret;
    }

    public void abort() {
        switch(state_) {
            case Idle:
                break ;

            case LookingForTag:
                break ;

            case DrivingToLocation:
                drive_to_.cancel() ;
                break ;
        }

        state_ = State.Idle;
        operation_ = null;
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        Gamepad gp = oi_.getGamePad() ;

        if (gp.isEnabled() == false && gp.isXPressed() && gp.isAPressed()) {
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
                break;

            case LookingForTag:
                lookingForTag() ;
                break;

            case DrivingToLocation:
                if (drive_to_.isDone() && shelf_collect_action_.isDone()) {
                    oi_.enableGamepad() ;
                    oi_.getGamePad().rumble(1.0, 2.0);
                    state_ = State.Idle;
                }
                break ;
        }

        if (state_ != orig) {
            MessageLogger logger = getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getLoggerID());
            logger.add("State Changes: " + orig.toString() + " -> " + state_.toString());
            logger.endMessage();
        }
    }

    private void lookingForTag() {
        if (limelight_.hasAprilTag(operation_.getAprilTag())) {
            oi_.disableGamepad() ;

            if (operation_.getAction() == RobotOperation.Action.Collect) {
                target_pose_ = locdata_.getGridPose(operation_.getAprilTag(), operation_.getSlot());
            }
            else {
                target_pose_ = locdata_.getLoadingStationPose(operation_.getSlot());
            }

            drive_to_ = new SwerveDriveToPoseAction(db_, target_pose_);
            db_.setAction(drive_to_);
            gpm_.setAction(shelf_collect_action_, true);

            state_ = State.DrivingToLocation;
        }
    }
}