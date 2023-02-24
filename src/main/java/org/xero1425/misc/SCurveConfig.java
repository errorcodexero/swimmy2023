package org.xero1425.misc ;

public class SCurveConfig {
    public final double jerk ;
    public final double maxv ;
    public final double maxa ;

    public SCurveConfig(double j, double a, double v) {
        jerk = j ;
        maxa = a ;
        maxv = v ;
    }
}
