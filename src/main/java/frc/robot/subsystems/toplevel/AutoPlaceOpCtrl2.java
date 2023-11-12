package frc.robot.subsystems.toplevel;

import java.util.ArrayList;
import java.util.List;

import org.xero1425.base.misc.XeroElapsedTimer;
import org.xero1425.base.subsystems.swerve.common.SwerveDrivePathAction;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.gpm.GPMPlaceAction;

public class AutoPlaceOpCtrl2 extends OperationCtrl {
    private enum State {
        Start,
        WaitForTag,
        DrivingToLocation,
        WaitForArm,
        DroppingPiece,
        Done,
        Error
    } ;

    private State state_ ;
    private double april_tag_action_threshold_ ;
    private GPMPlaceAction place_action_ ;
    private XeroElapsedTimer overall_timer_ ;
    private SwerveDrivePathAction drive_to_action_ ;
    private XeroElapsedTimer wait_for_arm_timer_ ;
    private DriverStation.Alliance alliance_ ;

    public AutoPlaceOpCtrl2(Swimmy2023RobotSubsystem sub, RobotOperation oper) throws Exception {
        super(sub, oper) ;

        state_ = State.Start ;
        april_tag_action_threshold_ = sub.getSettingsValue("april-tag-place-action-threshold").getDouble() ;
        april_tag_action_threshold_ = 2.0 ;
        place_action_ = new GPMPlaceAction(sub.getGPM(), oper.getLocation(), oper.getGamePiece(), false, false);
        overall_timer_ = new XeroElapsedTimer(sub.getRobot());
        wait_for_arm_timer_ = new XeroElapsedTimer(sub.getRobot());
        alliance_ = DriverStation.getAlliance() ;
    }

    @Override
    public void start() throws BadParameterTypeException, MissingParameterException {
        super.start();
        state_ = State.Start ;
    }

    @Override
    public void abort() {
        switch(state_) {
            case Start:
                break ;

            case WaitForTag:
                break ;

            case DrivingToLocation:
                getRobotSubsystem().getSwerve().cancelAction();
                getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
                break;

            case WaitForArm:
                break ;

            case DroppingPiece:
                break ;

            case Done:
                break ;

            case Error:
                break ;
        }        
    }    

    @Override
    public void run() throws BadParameterTypeException, MissingParameterException {
        State orig = state_ ;

        switch(state_) {
            case Start:
                stateIdle() ;
                break ;

            case WaitForTag:
                waitForTag() ;
                break ;

            case DrivingToLocation:
                drivingToLocation() ;
                break;

            case WaitForArm:
                waitForArm() ;
                break ;

            case DroppingPiece:
                droppingPiece() ;
                break ;

            case Done:
            case Error:
                break ;
        }

        if (state_ != orig) {
            MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
            logger.add("AutoPlaceOpCtrl2 State Changes: " + orig.toString() + " -> " + state_.toString());
            logger.endMessage();
        }
    }

    private void stateIdle() {
        state_ = State.WaitForTag ;
    }

    private void waitForTag() {
        int tag = getRobotSubsystem().getFieldData().getGridTag(Alliance.Invalid, getOper().getAprilTag());
        double dist = getRobotSubsystem().getLimeLight().distantToTag(tag) ;

        MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger();
        logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
        logger.add("Waiting For Tag");
        logger.add("tag", tag) ;
        logger.add("dist", dist) ;
        logger.endMessage();

        if (dist < april_tag_action_threshold_) {
            getRobotSubsystem().getOI().disableGamepad();
            getRobotSubsystem().getOI().getGamePad().rumble(1.0, 0.5);

            getRobotSubsystem().getGPM().setAction(place_action_, true);
            overall_timer_.reset() ;

            try {
                getRobotSubsystem().getSwerve().enableVision(false);
                drive_to_action_ = createDriveToLocationAction() ;
                getRobotSubsystem().getSwerve().setAction(drive_to_action_, true);
                state_ = State.DrivingToLocation ;   
            }
            catch(Exception ex) {
                logger.startMessage(MessageType.Error, getRobotSubsystem().getLoggerID());
                logger.add("exception creating drive to action - " + ex.getMessage());
                logger.endMessage() ;
                logger.logStackTrace(ex.getStackTrace());
                getRobotSubsystem().getSwerve().cancelAction();
                getRobotSubsystem().getSwerve().drive(new ChassisSpeeds());
                state_ = State.Error ;
                setDone() ;
            }
        }
    }

    private void drivingToLocation() {
        if (drive_to_action_.isDone()) {
            wait_for_arm_timer_.reset() ;
            state_ = State.WaitForArm ;
            getRobotSubsystem().getSwerve().enableVision(true);
        }
    }

    private void waitForArm() {
        if (place_action_.isReadyToDrop()) {
            MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Info) ;
            logger.add("Dropping game piece") ;
            logger.add("wait time", wait_for_arm_timer_.elapsed());
            logger.add("uppper arm", getRobotSubsystem().getGPM().getArm().getUpperSubsystem().getPosition());
            logger.add("lower arm", getRobotSubsystem().getGPM().getArm().getLowerSubsystem().getPosition());
            logger.endMessage();
            place_action_.dropGamePiece();
            state_ = State.DroppingPiece ;
        }
    }

    private void droppingPiece() {
        if (place_action_.isDone()) {
            state_ = State.Done ;

            getRobotSubsystem().getOI().enableGamepad();
            getRobotSubsystem().getOI().getGamePad().rumble(1.0, 0.5);

            MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger() ;
            logger.startMessage(MessageType.Debug, getRobotSubsystem().getLoggerID());
            logger.add("AutoPlaceOpCtrl duration: " + overall_timer_.elapsed());
            logger.endMessage();

            setDone();
        }
    }

    private SwerveDrivePathAction createDriveToLocationAction() throws BadParameterTypeException, MissingParameterException {
        final double endpos = 0.06;
        final double immdpos = 0.2 ;

        double maxa = 0.5;
        double maxv = 1.0 ;

        double sign = (alliance_ == Alliance.Red) ? 1.0 : -1.0 ;
        Pose2d oldend = getRobotSubsystem().getFieldData().getGridPose(Alliance.Invalid, getOper().getAprilTag(), getOper().getSlot());

        Pose2d end = new Pose2d(oldend.getX() + sign * endpos, oldend.getY(), oldend.getRotation());
        Translation2d immd = new Translation2d(oldend.getX() + -sign * immdpos, oldend.getY()) ;

        List<Translation2d> interior = new ArrayList<Translation2d>() ;
        interior.add(immd) ;

        MessageLogger logger = getRobotSubsystem().getRobot().getMessageLogger() ;
        logger.startMessage(MessageType.Info) ;
        logger.add("middle", oldend.getTranslation()) ;
        logger.add("end", end) ;
        logger.endMessage();

        return new SwerveDrivePathAction(getRobotSubsystem().getSwerve(), interior, end, 0.0, end.getRotation(), maxa, maxv);
    }
}
