package org.xero1425.simulator.models;

public class WheelSubModel {
    private SimMotorController motor_ ;
    private double ticks_per_meter_ ;
    private double meters_per_second_per_power_ ;
    private double position_ ;
    private double speed_ ;

    public WheelSubModel(SimMotorController motor, double ticks_per_meter, double meters_per_second_per_power) {
        motor_ = motor ;
        meters_per_second_per_power_ = meters_per_second_per_power ;

        ticks_per_meter_ = ticks_per_meter ;

        position_ = 0.0 ;
        speed_ = 0.0 ;
    }

    public void run(double dt) {
        double power = motor_.getPower() ;

        double dpos = meters_per_second_per_power_ * power * dt ;
        position_ += dpos ;
        speed_ = dpos / dt ;
   
        if (motor_.usesTicks()) {
            double ticks = position_ * ticks_per_meter_ ;
            motor_.setEncoder(ticks);
        }
    }

    public double getPosition() {
        return position_ ;
    }

    public double getSpeed() {
        return speed_ ;
    }
}
