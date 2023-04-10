package org.xero1425.misc;

public class SCurveProfile implements IMotionProfile {

    static private int instance = 0 ;

    private SCurveConfig config_ ;
    private double [] t_ ;
    private double [] v_ ;
    private double [] d_ ;
    private double dist_ ;
    private int myinst ;

    public SCurveProfile(SCurveConfig config) {
        config_= config ;
        t_ = new double[7] ;
        v_ = new double[6] ;
        d_ = new double[6] ;
        myinst = instance++ ;
    }

    public boolean update(double dist, double vi, double vf) {
        if (Math.abs(vi) > 1e-6) {
            return false ;
        }

        if (Math.abs(vf) > 1e-6) {
            return false ;
        }

        dist_ = dist ;
        return computeTimes(dist) ;
    }

    public double getAccel(double t) {
        double ret = 0 ;
        int seg = segment(t) ;
        double st = (seg == 0) ? 0.0 : sumOfT(seg);
        double dt =  t - st ;

        switch(seg) {
        case 0:
            ret = dt * config_.jerk ;
            break;
        case 1:
            ret = config_.maxa ;
            break;        
        case 2:
            ret = config_.maxa - dt * config_.jerk ;
            break;
        case 3:
            ret = 0.0 ;
            break;
        case 4:
            ret = -dt * config_.jerk ;
            break;
        case 5:
            ret = -config_.maxa ;
            break;
        case 6:
            ret = -config_.maxa + dt * config_.jerk ;
            break;
        }


        return ret ;
    }

    public double getVelocity(double t)  {
        double ret = 0 ;
        int seg = segment(t) ;
        double st = (seg == 0) ? 0.0 : sumOfT(seg);
        double dt =  t - st ;

        switch(seg) {
            case 0:
                ret = 0.5 * config_.jerk * dt * dt ;
                break;
            case 1:
                ret = v_[0] + config_.maxa * dt ;
                break;        
            case 2:
                ret = v_[1] + config_.maxa * dt + 0.5 * -config_.jerk * dt * dt ;
                break;
            case 3:
                ret = v_[2] ;
                break;
            case 4:
                ret = v_[3] + 0.5 * -config_.jerk * dt * dt ;
                break;
            case 5:
                ret = v_[4] + -config_.maxa * dt ;
                break;
            case 6:
                ret = v_[5] + -config_.maxa * dt + 0.5 * config_.jerk * dt * dt ;
                break;
        }

        if (myinst == 0)
            System.out.println("Velocity t=" + t + " seg " + seg + " st=" + st + " dt=" + dt + " vel=" + ret);

        return ret ;
    }

    public double getTotalTime()  {
        return t_[6] + t_[5] + t_[4] + t_[3] + t_[2] + t_[1] + t_[0] ;
    }

    public double getDistance(double t)  {
        double ret = 0 ;
        int seg = segment(t);
        double st = (seg == 0) ? 0.0 : sumOfT(seg);
        double dt =  t - st ;

        switch(segment(t)) {
            case 0:
                ret = distance(dt, 0.0, 0.0, 0.0, config_.jerk);
                break;
            case 1:
                ret = distance(dt, d_[0], v_[0], config_.maxa, 0.0);
                break;        
            case 2:
                ret = distance(dt, d_[1], v_[1], config_.maxa, -config_.jerk);
                break;
            case 3:
                ret = distance(dt, d_[2], v_[2], 0.0, 0.0);
                break;
            case 4:
                ret = distance(dt, d_[3], v_[3], 0.0, -config_.jerk);
                break;
            case 5:
                ret = distance(dt, d_[4], v_[4], -config_.maxa, 0.0);
                break;
            case 6:
                ret = distance(dt, d_[5], v_[5], -config_.maxa, config_.jerk);
                break;
        }
        
        return ret ;
    }

    private double distance(double t, double x0, double v0, double a0, double j) {
        return x0 + v0 * t + 0.5 * a0 * t * t + 1.0 / 6.0 * j * t * t * t ;
    }

    private double sumOfT(int seg) {
        double t = 0.0 ;

        for(int i = 0 ; i < seg ; i++) {
            t += t_[i] ;
        }

        return t ;
    }

    private boolean computeTimes(double dist) {
        v_[0] = config_.maxa * config_.maxa / 2.0 / config_.jerk ;
        v_[1] = config_.maxv - config_.maxa * config_.maxa / 2.0 / config_.jerk ;
        v_[2] = config_.maxv ;
        v_[3] = config_.maxv ;
        v_[4] = config_.maxv - config_.maxa * config_.maxa / 2.0 / config_.jerk ;
        v_[5] = config_.maxa * config_.maxa / 2.0 / config_.jerk ;

        t_[0] = config_.maxa / config_.jerk;
        t_[1] = (v_[1] - v_[0]) / config_.maxa ;
        t_[2] = config_.maxa / config_.jerk ;
        t_[3] = ((config_.maxa * config_.jerk * dist) - (config_.maxv * config_.maxa * config_.maxa) - (config_.jerk * config_.maxv * config_.maxv)) / (config_.jerk * config_.maxv * config_.maxa) ;
        t_[4] = config_.maxa / config_.jerk;
        t_[5] = (v_[4] - v_[5]) / config_.maxa ;
        t_[6] = config_.maxa / config_.jerk ;

        d_[0] = distance(t_[0], 0.0, 0.0, 0.0, config_.jerk);           // Distance at end of segment 1
        d_[1] = distance(t_[1], d_[0], v_[0], config_.maxa, 0.0) ;              // Distance at end of segment 2
        d_[2] = distance(t_[2], d_[1], v_[1], config_.maxa, -config_.jerk) ;        // Distance at end of segment 3
        d_[3] = distance(t_[3], d_[2], v_[2], 0.0, 0.0) ;                       // Distance at end of segment 4
        d_[4] = distance(t_[4], d_[3], v_[3], 0.0, -config_.jerk) ;             // Distance at end of segment 5
        d_[5] = distance(t_[5], d_[4], v_[4], -config_.maxa, 0.0) ;                     // Distance at end of segment 6

        return t_[3] >= 0.0 ;
    }

    private int segment(double t) {
        int ret = -1 ;
        double ttotal = 0.0 ;

        for(int i = 0 ; i < 7 ; i++) {
            ttotal += t_[i] ;
            if (t <= ttotal) {
                ret = i ;
                break ;
            }
        }

        return ret;
    }
}
