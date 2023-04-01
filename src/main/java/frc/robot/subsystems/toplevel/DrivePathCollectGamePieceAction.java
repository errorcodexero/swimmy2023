package frc.robot.subsystems.toplevel;

import org.xero1425.base.actions.Action;
import org.xero1425.base.misc.XeroTimer;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.arm.ArmStaggeredGotoAction;
import frc.robot.subsystems.gpm.GPMCollectAction;
import frc.robot.subsystems.toplevel.RobotOperation.GamePiece;

public class DrivePathCollectGamePieceAction extends Action {

    private enum State
    {
        Idle,
        DrivingPath,
        TrackingGamePiece,
        CollectGamePiece
    } ;

    private class GamePieceDetectInfo
    {
        public GamePiece type_ ;
        public double tx_ ;
        public double ty_ ;
        public double ta_;
    }

    private Swimmy2023RobotSubsystem sub_ ;
    private String pathname_ ;
    private double delay_ ;
    private SwerveHolonomicPathFollower pathfollow_;
    private State state_ ;
    private XeroTimer timer_ ;
    private XeroTimer start_collect_timer_ ;
    private ArmStaggeredGotoAction arm_pre_position_ ;
    private GPMCollectAction collect_action_ ;

    private double kp_ = 0.25 / 3.0 ;
    private double tracking_x_speed_ = 1.0 ;
    private double align_to_gp_delay_ = 1.0 ;
    private double track_to_collect_delay_ = 1.0 ;

    public DrivePathCollectGamePieceAction(Swimmy2023RobotSubsystem sub, String path, GamePiece gp, boolean setpose, double delay) throws Exception {
        super(sub.getRobot().getMessageLogger());
        sub_ = sub;
        pathname_ = path ;
        delay_ = delay ;

        pathfollow_ = new SwerveHolonomicPathFollower(sub.getSwerve(), path, setpose, 0.2);
        state_ = State.Idle ;

        arm_pre_position_ = new ArmStaggeredGotoAction(sub.getGPM().getArm(), "pre-extend-ground", false);
        collect_action_ = new GPMCollectAction(sub_.getGPM(), gp, true);
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        sub_.getSwerve().setAction(pathfollow_, true);
        state_ = State.DrivingPath ;

        start_collect_timer_ = new XeroTimer(sub_.getRobot(), "start-collect", delay_);
        start_collect_timer_.start();
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (start_collect_timer_.isExpired()) {
            //
            // Preposition the ARM to be ready to collect
            //
            sub_.getGPM().getArm().setAction(arm_pre_position_, true);
        }

        GamePieceDetectInfo info = getGamePieceInfo() ;
        if (state_ == State.DrivingPath) {
            if (info != null) {
                //
                // Ok, we see a game piece, switch to a drive to mode
                //
                state_ = State.TrackingGamePiece;
                timer_ = new XeroTimer(sub_.getRobot(), "DrivePathCollectGamePieceAction-1", align_to_gp_delay_);
                timer_.start();
            }
        }
        else if (state_ == State.TrackingGamePiece) {
            if (info == null) {
                //
                // We lost the target, stop the robot
                //
                sub_.getSwerve().drive(new ChassisSpeeds());
                state_ = State.Idle ;
                setDone();
            }
            else if (timer_.isExpired()) {
                timer_ = new XeroTimer(sub_.getRobot(), "DrivePathCollectGamePieceAction-2", track_to_collect_delay_);
                timer_.start();
                state_ = State.CollectGamePiece ;

                ChassisSpeeds speeds = new ChassisSpeeds(tracking_x_speed_, 0.0, 0.0) ;
                sub_.getSwerve().drive(speeds);

                sub_.getGPM().setAction(collect_action_, true);
            }
            else {
                ChassisSpeeds speeds = new ChassisSpeeds(tracking_x_speed_, kp_ * info.tx_, 0.0);
                sub_.getSwerve().drive(speeds);
            }
        }
        else if (state_ == State.CollectGamePiece) {
            if (timer_.isExpired()) {
                //
                // Our timer expired but we did not collect the game piece, stop so we
                // don't drive to the other side of the field.
                //
                sub_.getSwerve().drive(new ChassisSpeeds());
                state_ = State.Idle ;
                setDone();
            }
            else {

            }
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "DrivePathCollectGamePieceAction '" + pathname_ + "' " + delay_ ;
    }

    private GamePieceDetectInfo getGamePieceInfo() {
        return null;
    }
}
