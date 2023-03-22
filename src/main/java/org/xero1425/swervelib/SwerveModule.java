package org.xero1425.swervelib;

import org.xero1425.misc.MessageLogger;

public interface SwerveModule {
    double getDistance() ;

    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void synchronizeEncoders(MessageLogger logger, String which) ;

    void heartBeat(MessageLogger logger, String which);
}
