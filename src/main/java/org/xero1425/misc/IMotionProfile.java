package org.xero1425.misc;

public interface IMotionProfile {
    boolean update(double dist, double start_velocity, double end_velocity) ;
    double getAccel(double t) ;
    double getVelocity(double t) ;
    double getDistance(double t) ;
    double getTotalTime() ;
}
