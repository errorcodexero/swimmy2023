package org.xero1425.swervelib;

import org.xero1425.misc.MessageLogger;

public interface SteerController {
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    double synchronizeEncoders(MessageLogger logger, String which) ;

    void heartBeat(MessageLogger logger, String which) ;
}
