package org.xero1425.base.subsystems.swerve.common;

import org.xero1425.base.actions.Action;

import edu.wpi.first.math.geometry.Pose2d;

public class SwerveDriveSetPoseAction extends Action {

    private SwerveBaseSubsystem sub_ ;
    private Pose2d pose_ ;

    public SwerveDriveSetPoseAction(SwerveBaseSubsystem sub, Pose2d pose) {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;
        pose_ = pose ;
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        sub_.setPose(pose_) ;
    }

    @Override
    public String toString(int indent) {
        String ret = spaces(indent) + "SwerveDriveSetPoseAction " + pose_.toString();
        return ret ;
    }
}
