package org.xero1425.base.subsystems.swerve.common;

public class SwerveDriveXPatternAction extends SwerveDriveAction {

    static final double[] angles_ = { -45.0, 45.0, 45.0, -45.0 } ;
    static final double[] powers_ = { 0.0, 0.0, 0.0, 0.0 };

    public SwerveDriveXPatternAction(SwerveBaseSubsystem sub) {
        super(sub);
    }

    @Override
    public void start() {
        getSubsystem().setRawTargets(true, angles_, powers_);
    }

    @Override
    public void run() {

    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "SwerveDriveXPatternAction" ;
    }
}
