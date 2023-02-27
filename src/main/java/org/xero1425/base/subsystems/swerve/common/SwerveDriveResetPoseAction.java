package org.xero1425.base.subsystems.swerve.common;

public class SwerveDriveResetPoseAction extends SwerveDriveAction {
    public SwerveDriveResetPoseAction(SwerveBaseSubsystem sub) {
        super(sub);
    }

    @Override
    public void start() throws Exception {
        super.start() ;
        getSubsystem().resetPose(false);
        setDone();
    }

    @Override
    public void run() throws Exception {
        super.run() ;
    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveDriveResetPoseAction" ;
    }
}
