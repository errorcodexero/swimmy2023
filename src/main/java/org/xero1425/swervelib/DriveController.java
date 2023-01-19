package org.xero1425.swervelib;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getDistance() ;
}
