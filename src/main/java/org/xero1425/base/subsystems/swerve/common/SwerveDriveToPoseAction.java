package org.xero1425.base.subsystems.swerve.common;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

public class SwerveDriveToPoseAction extends SwerveDriveAction {
    private HolonomicDriveController ctrl_ ;
    private Trajectory trajectory_ ;
    private Pose2d target_position_ ;
    private double trajectory_start_time_ ;
    
    public SwerveDriveToPoseAction(SwerveBaseSubsystem subsys, Pose2d pose2d) {
        super(subsys) ;

        target_position_ = pose2d ;
    }

    @Override
    public void start() {
        trajectory_ = getSubsystem().createTrajectory(target_position_) ;
        trajectory_start_time_ = getSubsystem().getRobot().getTime() ;
    }

    @Override
    public void run() {
        Trajectory.State st = trajectory_.sample(getSubsystem().getRobot().getTime() - trajectory_start_time_) ;
        ChassisSpeeds speed = ctrl_.calculate(getSubsystem().getPose(), st, target_position_.getRotation()) ;
        getSubsystem().drive(speed) ;
        if (ctrl_.atReference()) {
            setDone() ;
        }
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveDriveToPoseAction " + target_position_.getTranslation().toString() + ", " + 
                target_position_.getRotation().getDegrees() + " deg" ;
    }
}
