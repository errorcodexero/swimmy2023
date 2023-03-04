package frc.robot.subsystems.toplevel;

import java.util.ArrayList;
import java.util.List;

import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.motorsubsystem.MotorEncoderPowerAction;
import org.xero1425.base.subsystems.swerve.common.SwerveDrivePathAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.gpm.GPMPlaceAction;
import frc.robot.subsystems.swerve.SwerveLinearAlignAction;
import frc.robot.subsystems.toplevel.RobotOperation.Action;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;
import frc.robot.subsystems.toplevel.RobotOperation.Location;

public class AutoPlaceOpCtrl extends OperationCtrl {

    private final boolean AddAlignStep = false ;

    private enum State {
        Idle,
        LookingForTag,
        WaitingOnVision,
        DrivingToLocation,
        AlignRobot,
        AlignWheels,
        DriveForward,
        WaitingOnArm,
        SettlingDelay,
        DroppingPiece,
    }

    private double april_tag_action_threshold_ ;
    private State state_ ;
    private Pose2d target_pose_ ;

    private SwerveDrivePathAction drive_to_action_ ;
    private GPMPlaceAction place_action_ ;
    private MotorEncoderPowerAction spit_cube_action_ ;
    private SwerveLinearAlignAction align_action_ ;

    private XeroTimer vision_timer_ ;
    private XeroTimer forward_timer_ ;
    private XeroTimer settling_timer_ ;
    private XeroTimer wheels_timer_ ;
    
    public AutoPlaceOpCtrl(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws BadParameterTypeException, MissingParameterException {
        super(sub, oper) ;

        april_tag_action_threshold_ = sub.getSettingsValue("april-tag-action-threshold").getDouble() ;
        state_ = State.Idle ;

        vision_timer_ = new XeroTimer(sub.getRobot(), "vision/timer", 0.5);
        settling_timer_ = new XeroTimer(sub.getRobot(), "settling", 0.25) ;
        align_action_ = new SwerveLinearAlignAction(getRobotSubsystem().getSwerve(), getRobotSubsystem().getLimeLight()) ;

        if (oper.getAction() == Action.Place && oper.getGamePiece() == GamePiece.Cube && oper.getLocation() == Location.Bottom) {
            double power = getRobotSubsystem().getSettingsValue("spit-cube:power").getDouble();
            double duration = getRobotSubsystem().getSettingsValue("spit-cube:duration").getDouble();
            spit_cube_action_ = new MotorEncoderPowerAction(getRobotSubsystem().getGPM().getGrabber().getSpinSubsystem(), power, duration);
        }
        else {
            place_action_ = new GPMPlaceAction(sub.getGPM(), oper.getLocation(), oper.getGamePiece(), false);
        }

        forward_timer_ = new XeroTimer(sub.getRobot(), "forward", 1.0) ;
        wheels_timer_ = new XeroTimer(sub.getRobot(), "wheels", 0.2) ;
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

            case WaitingOnVision:
                stateWaitingForVision() ;
                break ;                

            case DrivingToLocation:
                stateDrivingToLocation() ;
                break;

            case AlignRobot:
                stateAlignRobot() ;
                break ;

            case AlignWheels:
                stateAlignWheels() ;
                break ;

            case DriveForward:
                stateDriveForward() ;
                break ;

            case WaitingOnArm:
                stateWaitingOnArm() ;
                break;

            case SettlingDelay:
                stateSettlingDelay() ;
                break ;                

            case DroppingPiece:
                stateDroppingPiece() ;
                break;
        }

        if (state_ != orig) {
            MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
            logger.add("AutoPlaceOpCtrl State Changes: " + orig.toString() + " -> " + state_.toString());
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

            case WaitingOnVision:
                break ;

            case AlignWheels:
                break ;

            case AlignRobot:
                getRobotSubsystem().getOI().enableGamepad() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                drive_to_action_.cancel() ;
                getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
                getRobotSubsystem().getLimeLight().setPipeline(0);
                break;

            case DrivingToLocation:
                getRobotSubsystem().getOI().enableGamepad() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                drive_to_action_.cancel() ;
                getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
                break ;                

            case DriveForward:
                drive_to_action_.cancel() ;
                getRobotSubsystem().getOI().enableGamepad() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
                break ;

            case WaitingOnArm:
                getRobotSubsystem().getOI().enableGamepad() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                break;

            case SettlingDelay:                
                getRobotSubsystem().getOI().enableGamepad() ;
                getRobotSubsystem().getSwerve().enableVision(true);
                break;

            case DroppingPiece:
                getRobotSubsystem().getOI().enableGamepad() ;
                if (place_action_ != null) {
                    place_action_.cancel();
                }
                break;
        }

        setDone();
        state_ = State.Idle;        
    }       

    private void stateIdle() {
        state_ = State.LookingForTag ;
    }

    private List<Pose2d> computeDrivePathPoints(Pose2d robotpos, Pose2d destpos) {
        List<Pose2d> ret = new ArrayList<Pose2d>();

        double dy = destpos.getY() - robotpos.getY() ;
        double dx = destpos.getX() - robotpos.getX() ;
        double angle = Math.atan2(dy, dx);

        Pose2d p0 = new Pose2d(robotpos.getX(), robotpos.getY(), Rotation2d.fromRadians(angle)) ;
        Pose2d p1 = new Pose2d(destpos.getX(), destpos.getY(), Rotation2d.fromRadians(angle)) ;

        ret.add(p0);
        ret.add(p1) ;

        return ret;
    }

    private void stateLookingForTag() throws BadParameterTypeException, MissingParameterException {
        int tag = getRobotSubsystem().getFieldData().getGridTag(Alliance.Invalid, getOper().getAprilTag());
        double dist = getRobotSubsystem().getLimeLight().distantToTag(tag) ;

        MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
        logger.add("Waiting For Tag");
        logger.add("tag", tag) ;
        logger.add("dist", dist) ;
        logger.endMessage();

        if (dist < april_tag_action_threshold_) {
            getRobotSubsystem().getSwerve().cancelAction();
            getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
            getRobotSubsystem().getOI().disableGamepad();
            getRobotSubsystem().getOI().getGamePad().rumble(1.0, 0.5);
            state_ = State.WaitingOnVision ;
            vision_timer_.start() ;
        }
    }

    private void stateWaitingForVision() throws BadParameterTypeException, MissingParameterException {
        int tag = getRobotSubsystem().getFieldData().getGridTag(Alliance.Invalid, getOper().getAprilTag());
        double dist = getRobotSubsystem().getLimeLight().distantToTag(tag) ;

        MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger();
        if (vision_timer_.isExpired()) {

            getRobotSubsystem().getSwerve().enableVision(false);
            target_pose_ = getRobotSubsystem().getFieldData().getGridPose(Alliance.Invalid, getOper().getAprilTag(), getOper().getSlot());

            List<Pose2d> pts = computeDrivePathPoints(getRobotSubsystem().getSwerve().getPose(), target_pose_);
            List<Translation2d> interior = new ArrayList<Translation2d>() ;
            drive_to_action_ = new SwerveDrivePathAction(getRobotSubsystem().getSwerve(), pts.get(0), interior, pts.get(1), target_pose_.getRotation(), 1.0, 1.0);

            getRobotSubsystem().getSwerve().setAction(drive_to_action_, true);
            if (place_action_ != null) {
                getRobotSubsystem().getGPM().setAction(place_action_, true);
            }

            if (AddAlignStep) {
                getRobotSubsystem().getLimeLight().setPipeline(1);
            }
            state_ = State.DrivingToLocation ;
        }
        else {
            logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
            logger.add("Waiting On Vision") ;
            logger.add("tag", tag) ;
            logger.add("dist", dist) ;
            logger.add("vision", getRobotSubsystem().getLimeLight().getBlueBotPose().toPose2d());
            logger.add("db", getRobotSubsystem().getSwerve().getPose());
            logger.endMessage();
        }
    }

    private void stateDrivingToLocation() {
        if (drive_to_action_.isDone()) {
            if (AddAlignStep) {
                getRobotSubsystem().getSwerve().setAction(align_action_, true) ;
                state_ = State.AlignRobot ;
            } else {
                double [] angles = new double[] { 0.0, 0.0, 0.0, 0.0} ;
                double [] power = new double[] { 0.0, 0.0, 0.0, 0.0} ;
    
                getRobotSubsystem().getLimeLight().setPipeline(0);
    
                getRobotSubsystem().getSwerve().setRawTargets(true, angles, power) ;
                state_ = State.AlignWheels ;
                wheels_timer_.start() ;
            }
        }
    }

    private void stateAlignRobot() {
        if (align_action_.isDone()) {
            double [] angles = new double[] { 0.0, 0.0, 0.0, 0.0} ;
            double [] power = new double[] { 0.0, 0.0, 0.0, 0.0} ;

            getRobotSubsystem().getLimeLight().setPipeline(0);

            getRobotSubsystem().getSwerve().setRawTargets(true, angles, power) ;
            state_ = State.AlignWheels ;
            wheels_timer_.start() ;
        }
    }

    private void stateAlignWheels() {
        if (wheels_timer_.isExpired()) {
            ChassisSpeeds speed ;            
            double xspeed = 0.5 ;
            speed = new ChassisSpeeds(xspeed, 0.0, 0.0) ;
            getRobotSubsystem().getSwerve().drive(speed) ;
            forward_timer_.start() ;
            state_ = State.DriveForward;
        }
    }

    private void stateDriveForward() {
        if (forward_timer_.isExpired()) {
            getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
            getRobotSubsystem().getSwerve().enableVision(true);
            settling_timer_.start() ;
            state_ = State.SettlingDelay ;
        }
    }

    private void stateSettlingDelay() {
        if (settling_timer_.isExpired()) {
            state_ = State.WaitingOnArm;
        }
    }

    private void stateWaitingOnArm() {
        if (place_action_ != null && place_action_.isReadyToDrop()) {
            MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Info) ;
            logger.add("Dropping game piece") ;
            logger.add("uppper arm", getRobotSubsystem().getGPM().getArm().getUpperSubsystem().getPosition());
            logger.add("lower arm", getRobotSubsystem().getGPM().getArm().getLowerSubsystem().getPosition());
            logger.endMessage();
            place_action_.dropGamePiece();
            state_ = State.DroppingPiece ;
        }
        else if (spit_cube_action_ != null) {
            getRobotSubsystem().getGPM().getGrabber().getSpinSubsystem().setAction(spit_cube_action_, true);
            state_ = State.DroppingPiece ;
        }
    }

    private void stateDroppingPiece() {
        if (place_action_ != null && place_action_.isDone()) {
            state_ = State.Idle ;

            if (place_action_.isDropComplete()) {
                getRobotSubsystem().getOI().enableGamepad();
                getRobotSubsystem().getOI().getGamePad().rumble(1.0, 0.5);
            }
            setDone();
        }
        else if (spit_cube_action_ != null && spit_cube_action_.isDone()) {
            state_ = State.Idle ;
            getRobotSubsystem().getOI().enableGamepad();
            getRobotSubsystem().getOI().getGamePad().rumble(1.0, 0.5);
            setDone();
        }
    }
}
