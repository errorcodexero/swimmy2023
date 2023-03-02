package org.xero1425.swervelib;

public interface SwerveModule {
    double getDistance() ;

    double getDriveVelocity();

    double getSteerAngle();

    double resetSteerEncoders();

    void set(double driveVoltage, double steerAngle);
}
