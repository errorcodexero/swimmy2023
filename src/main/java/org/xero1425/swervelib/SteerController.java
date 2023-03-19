package org.xero1425.swervelib;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    double synchronizeEncoders() ;
}
