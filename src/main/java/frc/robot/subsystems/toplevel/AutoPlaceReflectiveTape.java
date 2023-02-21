package frc.robot.subsystems.toplevel;

import org.xero1425.base.IVisionAlignmentData.CamMode;
import org.xero1425.base.IVisionAlignmentData.LedMode;
import org.xero1425.base.subsystems.swerve.common.SwerveAlignDriveBaseAction;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveToPoseAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.gpm.GPMPlaceAction;

public class AutoPlaceReflectiveTape extends OperationCtrl {

    static final int AprilTagPipeLine = 0 ;
    static final int RetroTapePipeLine = 1 ;

    private enum State {
        Idle,
        LookingForTag,
        DrivingToLocation,
        AlignDriveBase,
        ForwardDriveBase,
        DroppingPiece
    }

    private double april_tag_action_threshold_ ;
    private double tape_align_margin_ ;
    private double drive_forward_distance_ ;
    private State state_ ;
    private Pose2d target_pose_ ;

    private SwerveDriveToPoseAction drive_to_action_ ;
    private GPMPlaceAction place_action_ ;
    private SwerveAlignDriveBaseAction align_action_;
    private SwerveDriveToPoseAction forward_action_;
    
    public AutoPlaceReflectiveTape(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws BadParameterTypeException, MissingParameterException {
        super(sub, oper) ;

        april_tag_action_threshold_ = sub.getSettingsValue("april-tag-action-threshold").getDouble() ;
        tape_align_margin_ = sub.getSettingsValue("tape-align-margin").getDouble() ;
        drive_forward_distance_ = sub.getSettingsValue("drive-forward-distance").getDouble() ;

        state_ = State.Idle ;

        place_action_ = new GPMPlaceAction(sub.getGPM(), oper.getLocation(), oper.getGamePiece(), false);
        align_action_ = new SwerveAlignDriveBaseAction(sub.getSwerve(), sub.getLimeLight(), 3.0);
    }

    @Override
    public void start() throws BadParameterTypeException, MissingParameterException {
        super.start();
        state_ = State.Idle ;
    }

    @Override
    public void run() throws BadParameterTypeException, MissingParameterException {
        State orig = state_ ;

        switch(state_) {
            case Idle:
                stateIdle() ;
                break ;

            case LookingForTag:
                stateLookingForTag() ;
                break ;

            case DrivingToLocation:
                stateDrivingToLocation() ;
                break;

            case AlignDriveBase:
                stateAlignDriveBase();
                break;

            case ForwardDriveBase:
                stateForwardDriveBase();

            case DroppingPiece:
                stateDroppingPiece() ;
                break;
        }

        if (state_ != orig) {
            MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
            logger.add("AutoCollectOpCtrl State Changes: " + orig.toString() + " -> " + state_.toString());
            logger.endMessage();
        }
    }

    @Override
    public void abort() throws BadParameterTypeException, MissingParameterException {
        switch(state_) {
            case Idle:
                break ;

            case LookingForTag:
                break ;

            case DrivingToLocation:
                getRobotSubsystem().getOI().enableGamepad() ;
                getRobotSubsystem().getOI().getGamePad().rumble(1.0, 2.0);
                drive_to_action_.cancel() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                break ;

            case AlignDriveBase:
                getRobotSubsystem().getLimeLight().setPipeline(AprilTagPipeLine);
                getRobotSubsystem().getLimeLight().setCamMode(CamMode.VisionProcessing);
                getRobotSubsystem().getLimeLight().setLedMode(LedMode.ForceOff);
                getRobotSubsystem().getSwerve().cancelAction();
                break;

            case ForwardDriveBase:
                getRobotSubsystem().getSwerve().cancelAction();
                break;

            case DroppingPiece:
                getRobotSubsystem().getOI().enableGamepad() ;
                getRobotSubsystem().getOI().getGamePad().rumble(1.0, 2.0);
                place_action_.cancel();
                break;
        }

        setDone();
        state_ = State.Idle;        
    }       

    private void stateIdle() {
        state_ = State.LookingForTag ;
    }

    private void stateLookingForTag() throws BadParameterTypeException, MissingParameterException {
        int tag = getRobotSubsystem().getFieldData().getGridTag(getOper().getAprilTag());

        if (getRobotSubsystem().getLimeLight().distantToTag(tag) < april_tag_action_threshold_) {
            getRobotSubsystem().getOI().disableGamepad();
            getRobotSubsystem().getOI().getGamePad().rumble(1.0, 2.0);

            getRobotSubsystem().getSwerve().enableVision(false);

            target_pose_ = getRobotSubsystem().getFieldData().getLoadingStationPose(getOper().getSlot());

            if (DriverStation.getAlliance() == Alliance.Blue) {
                target_pose_ = new Pose2d(target_pose_.getX() + tape_align_margin_, target_pose_.getY(), target_pose_.getRotation());
            } else {
                target_pose_ = new Pose2d(target_pose_.getX() - tape_align_margin_, target_pose_.getY(), target_pose_.getRotation());                
            }

            drive_to_action_ = new SwerveDriveToPoseAction(getRobotSubsystem().getSwerve(), target_pose_);
            getRobotSubsystem().getSwerve().setAction(drive_to_action_);
            getRobotSubsystem().getGPM().setAction(place_action_);
            state_ = State.DrivingToLocation ;
        }
    }

    private void stateDrivingToLocation() {
        if (drive_to_action_.isDone()) {
            getRobotSubsystem().getLimeLight().setPipeline(RetroTapePipeLine);
            getRobotSubsystem().getLimeLight().setCamMode(CamMode.VisionProcessing);
            getRobotSubsystem().getLimeLight().setLedMode(LedMode.ForceOn);
            getRobotSubsystem().getSwerve().setAction(align_action_);
            state_ = State.AlignDriveBase;
        }
    }

    private void stateAlignDriveBase() throws BadParameterTypeException, MissingParameterException {
        if (align_action_.isDone()) {
            getRobotSubsystem().getLimeLight().setPipeline(AprilTagPipeLine);
            getRobotSubsystem().getLimeLight().setCamMode(CamMode.VisionProcessing);
            getRobotSubsystem().getLimeLight().setLedMode(LedMode.ForceOff);
            forward_action_ = new SwerveDriveToPoseAction(getRobotSubsystem().getSwerve(), getForwardPose());
            getRobotSubsystem().getSwerve().setAction(forward_action_);
        }
    }

    private void stateForwardDriveBase() {
        if (place_action_.isReadyToDrop() && forward_action_.isDone()) {
            place_action_.dropGamePiece();
            state_ = State.DroppingPiece;            
        }
    }

    private void stateDroppingPiece() {
        if (place_action_.isDone()) {
            state_ = State.Idle ;
            setDone();
        }
    }

    private Pose2d getForwardPose() {
        Pose2d ret = null;
        Pose2d here = getRobotSubsystem().getSwerve().getPose() ;

        if (DriverStation.getAlliance() == Alliance.Blue) {
            ret = new Pose2d(here.getX() + drive_forward_distance_, here.getY(), here.getRotation());
        }
        else {
            ret = new Pose2d(here.getX() - drive_forward_distance_, here.getY(), here.getRotation());
        }

        return ret;
    }
}
