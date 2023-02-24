package org.xero1425.misc;

public class SCurveProfile implements IMotionProfile {

    private SCurveConfig config_ ;
    private TrapezoidalProfile first_ ;
    private TrapezoidalProfile second_ ;
    private double [] t_ ;
    private double dist_ ;

    public SCurveProfile(SCurveConfig config) {
        config_= config ;
        t_ = new double[7] ;
    }

    private double getDist(double t, double j, double a, double v) {
        return 1.0 / 6.0 * j * t * t * t + 0.5 * a * t * t + v * t ;
    }

    private double getVelocity(double t, double j, double a, double v) {
        return 0.5 * j * t * t + a * t + v ;
    }

    private double getDist(int seg) {
        double dist = 0.0 ;

        switch(seg) {
            case 1:
            case 3:
            case 5:
            case 7:
                dist = getDist(t_[0], config_.jerk, 0.0, 0.0) ;
                break ;
            case 2:
            case 6:
                dist = getDist(t_[1], 0.0, first_.getVelocity(t_[0]), first_.getDistance(t_[0])) ;
                break ;
            case 4:
                dist = dist_ - getDist(1) - getDist(2) - getDist(3) - getDist(5) - getDist(6) - getDist(7) ;
                break ;
        }

        return dist ;
    }

    public boolean update(double dist, double vi, double vf) {
        if (Math.abs(vi) > 1e-6) {
            return false ;
        }

        if (Math.abs(vf) > 1e-6) {
            return false ;
        }

        dist_ = dist ;
        TrapezoidalProfileConfig cfg = new TrapezoidalProfileConfig(config_.jerk, -config_.jerk, config_.maxa);
        first_ = new TrapezoidalProfile(cfg);
        second_ = new TrapezoidalProfile(cfg);

        first_.update(config_.maxv, 0.0, 0.0) ;
        t_[0] = first_.getTimeAccel();
        t_[1] = first_.getTimeCruise();
        t_[2] = first_.getTimeDecel();

        second_.update(config_.maxv, 0.0, 0.0);
        t_[4] = second_.getTimeAccel();
        t_[5] = second_.getTimeCruise();
        t_[6] = second_.getTimeDecel();

        double dtotal = getDist(1) + getDist(2) + getDist(3) + getDist(5) + getDist(6) + getDist(7) ;

        dtotal *= 2.0 ;
        t_[3] = (dist - dtotal) / config_.maxv;

        return true;
    }

    public double getAccel(double t) {
        double ret = 0.0 ;

        if (t < t_[0]) {
            ret = config_.jerk * t ;
        }
        else if (t < t_[1] + t_[0]) {
            ret = config_.maxa ;
        }
        else if (t < t_[2] + t_[1] + t_[0]) {
            ret = config_.maxa - config_.jerk * (t - t_[1] - t_[0]) ;
        }
        else if (t < t_[3] + t_[2] + t_[1] + t_[0]) {
            ret = 0.0 ;
        }
        else if (t < t_[4] + t_[3] + t_[2] + t_[1] + t_[0]) {
            ret = -config_.jerk * (t - t_[3] - t_[2] - t_[1] - t_[0]) ;
        }
        else if (t < t_[5] + t_[4] + t_[3] + t_[2] + t_[1] + t_[0]) {
            ret = -config_.maxa ;
        }
        else if (t < t_[6] + t_[5] + t_[4] + t_[3] + t_[2] + t_[1] + t_[0]) {
            ret = -config_.maxa + config_.jerk *  (t - t_[5] - t_[4] - t_[3] - t_[2] - t_[1] - t_[0]) ;
        }

        return ret ;
    }

    public double getVelocity(double t)  {
        double ret = 0.0 ;

        if (t <= t_[0]) {
            ret = getVelocity(t, config_.jerk, 0.0, 0.0);
        }
        else if (t <= t_[1] + t_[0]) {
            double tt = t_[0] ;
            ret = getVelocity(t - tt, 0.0, getAccel(tt), getVelocity(tt));
        }
        else if (t <= t_[2] + t_[1] + t_[0]) {
            double tt = t_[0] + t_[1] ;
            ret = getVelocity(t - tt, -config_.jerk, getAccel(tt), getVelocity(tt));
        }
        else if (t <= t_[3] + t_[2] + t_[1] + t_[0]) {
            ret = config_.maxv ;
        }
        else if (t <= t_[4] + t_[3] + t_[2] + t_[1] + t_[0]) {
            double tt = t_[0] + t_[1] + t_[2] + t_[3];
            ret = getVelocity(t - tt, -config_.jerk, 0.0, config_.maxv) ;
        }
        else if (t <= t_[5] + t_[4] + t_[3] + t_[2] + t_[1] + t_[0]) {
            double tt = t_[0] + t_[1] + t_[2] + t_[3] + t_[4];
            ret = getVelocity(t - tt, 0.0, getAccel(tt), getVelocity(tt));
        }
        else if (t <= t_[6] + t_[5] + t_[4] + t_[3] + t_[2] + t_[1] + t_[0]) {
            double tt = t_[0] + t_[1] + t_[2] + t_[3] + t_[4] + t_[5];
            ret = getVelocity(t - tt, config_.jerk, -config_.maxa, getVelocity(tt)) ;
        }

        return ret ;
    }

    public double getDistance(double t)  {
        double ret = 0.0 ;

        if (t <= t_[0]) {
            ret = getDist(t, config_.jerk, 0.0, 0.0);
        }
        else if (t <= t_[1] + t_[0]) {
            double tt = t_[0] ;
            ret = getDist(t - tt, 0.0, getAccel(tt), getVelocity(tt)) + getDistance(tt);
        }
        else if (t <= t_[2] + t_[1] + t_[0]) {
            double tt = t_[0] + t_[1];
            ret = getDist(t - tt, -config_.jerk, getAccel(tt), getVelocity(tt)) + getDistance(tt);
        }
        else if (t <= t_[3] + t_[2] + t_[1] + t_[0]) {
            double tt = t_[0] + t_[1] + t_[2] ;
            ret = getDist(t - tt, 0.0, getAccel(tt), getVelocity(tt)) + getDistance(tt);
        }
        else if (t <= t_[4] + t_[3] + t_[2] + t_[1] + t_[0]) {
            double tt = t_[0] + t_[1] + t_[2] + t_[3];
            ret = getDist(t - tt, -config_.jerk, getAccel(tt), getVelocity(tt)) + getDistance(tt);
        }
        else if (t <= t_[5] + t_[4] + t_[3] + t_[2] + t_[1] + t_[0]) {
            double tt = t_[0] + t_[1] + t_[2] + t_[3] + t_[4];
            ret = getDist(t - tt, 0.0, getAccel(tt), getVelocity(tt)) + getDistance(tt);
        }
        else if (t <= t_[6] + t_[5] + t_[4] + t_[3] + t_[2] + t_[1] + t_[0]) {
            double tt = t_[0] + t_[1] + t_[2] + t_[3] + t_[4] + t_[5] ;
            ret = getDist(t - tt, config_.jerk, getAccel(tt), getVelocity(tt)) + getDistance(tt);
        }

        return ret ;
    }

    public double getTotalTime()  {
        return t_[6] + t_[5] + t_[4] + t_[3] + t_[2] + t_[1] + t_[0] ;
    }

}
