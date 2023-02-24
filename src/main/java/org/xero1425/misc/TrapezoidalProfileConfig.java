package org.xero1425.misc;

public class TrapezoidalProfileConfig {
    public final double accel ;
    public final double decel ;
    public final double velocity ;

    public TrapezoidalProfileConfig(double a, double d, double v) {
        accel = a ;
        decel = d ;
        velocity = v ;
    }
}
