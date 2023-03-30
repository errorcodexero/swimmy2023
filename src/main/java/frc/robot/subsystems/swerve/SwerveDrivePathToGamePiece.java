package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.xero1425.base.subsystems.swerve.common.SwerveBaseSubsystem;
import org.xero1425.base.subsystems.swerve.common.SwerveDriveAction;
import org.xero1425.base.subsystems.swerve.common.SwerveHolonomicPathFollower;
import org.xero1425.base.subsystems.vision.LimeLightSubsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class SwerveDrivePathToGamePiece extends SwerveDriveAction {
    private SwerveHolonomicPathFollower drive_path_ ;
    private SwerveDriveSteerToTarget steer_target_ ;
    private LimeLightSubsystem limelight_ ;
    private int prev_pipeline_ ;
    private int pipeline_ ;
    private boolean in_path_ ;

    public SwerveDrivePathToGamePiece(LimeLightSubsystem ll, int pipeline, SwerveBaseSubsystem swerve,String pathname, boolean setpose, double endtime, Supplier<Boolean> isAtTarget) throws BadParameterTypeException, MissingParameterException {
        super(swerve) ;

        in_path_ = true ;
        limelight_ = ll ;
        pipeline = pipeline_ ;
        drive_path_ = new SwerveHolonomicPathFollower(swerve, pathname, setpose, endtime);
        steer_target_ = new SwerveDriveSteerToTarget(ll, swerve, isAtTarget);
    }

    @Override
    public void start() throws Exception {
        super.start() ;

        in_path_ = true ;
        prev_pipeline_ = limelight_.getPipeLine();
        limelight_.setPipeline(pipeline_) ;
        getSubsystem().setAction(drive_path_);
    }

    @Override
    public void run() throws Exception {
        super.run() ;

        if (in_path_) {
            if (limelight_.isTargetDetected()) {
                getSubsystem().setAction(steer_target_);
            }
        }
        else {
            if (steer_target_.isDone()) {
                setDone() ;
            }
        }
    }

    @Override
    public void cancel() {
        super.cancel() ;
        limelight_.setPipeline(prev_pipeline_);
    }

    @Override
    public String toString(int indent) {
        return "SwerveDrivePathToGamePiece '" + drive_path_.getPathName() + "'" ;
    }
}
