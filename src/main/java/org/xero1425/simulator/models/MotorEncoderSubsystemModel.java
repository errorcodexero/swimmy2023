package org.xero1425.simulator.models;

import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;

public class MotorEncoderSubsystemModel extends SimulationModel {
    SimMotorController motor_ ;
    double rps_per_volt_per_time_ ;
    double ticks_per_rev_ ;
    double revs_ ;

    public MotorEncoderSubsystemModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;

        revs_ = 0.0 ;
    }   

    @Override
    public boolean create() {
        motor_ = new SimMotorController(this, getModelName()) ;

        if (!motor_.createMotor())
            return false ;

        try {
            rps_per_volt_per_time_ = getDoubleProperty("rps-per-volt-per-time") ;
            ticks_per_rev_ = getDoubleProperty("ticks-per-rev") ;
        }
        catch(Exception ex) {
            return false ;
        }

        setCreated();
        return true ;
    }

    @Override
    public void run(double dt) {
        
        double power = motor_.getPower() ;
        double rps = rps_per_volt_per_time_ * power * dt ;

        if (power > 0.1) {
            System.out.println("Here") ;
        }

        revs_ += rps ;

        if (motor_.usesTicks()) {
            double ticks = revs_ *  ticks_per_rev_ ;
            motor_.setEncoder(ticks);
        }
    }

    @Override
    public boolean processEvent(String name, SettingsValue v) {
        return true ;
    }
}
