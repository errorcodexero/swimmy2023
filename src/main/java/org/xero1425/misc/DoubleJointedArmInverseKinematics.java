package org.xero1425.misc;


public class DoubleJointedArmInverseKinematics {
    private double l1_ ;
    private double l2_ ;

    public DoubleJointedArmInverseKinematics(double l1, double l2) {
        l1_ = l1 ;
        l2_ = l2 ;
    }

    public double[] angles(double x, double y) {
        double [] ret = new double[2] ;

        double dist = Math.sqrt(x * x + y * y) ;
        double d1 = Math.atan2(y, x) ;

        double d2 = lawOfCosines(dist, l1_, l2_) ;

        ret[0] = d1 + d2 ;

        ret[1] = lawOfCosines(l1_, l2_, dist) ;

        return ret ;
    }

    private double lawOfCosines(double a, double b, double c) {
        return Math.acos((a * a + b * b - c * c) / (2 * a * b)) ;
    }
}
