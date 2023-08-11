package frc.models;

import org.xero1425.base.motors.MotorFactory;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

public class ArmModel extends SimulationModel {
    private double first_ticks_per_second_per_volt_ ;
    private double first_angle_ ;

    private double second_ticks_per_second_per_volt_ ;
    private double second_angle_ ;

    private TalonFXSimCollection first_ ;
    private TalonFXSimCollection second_ ;

    public ArmModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;

    }

    @Override
    public boolean create() {
        boolean ret = true ;

        try {
            int canid1 = getProperty("first:motor:canid").getInteger();
            int canid2 = getProperty("second:motor:canid").getInteger();
            String bus1 = getProperty("first:motor:bus").getString() ;
            String bus2 = getProperty("first:motor:bus").getString() ;

            first_ = getEngine().getRobot().getMotorFactory().getFXSimCollection(bus1, canid1) ;
            second_ = getEngine().getRobot().getMotorFactory().getFXSimCollection(bus2, canid2) ;

            setCreated(); 
        }
        catch(Exception ex) {
            ret = false ;
        }

        return ret ;
    }

    @Override
    public boolean processEvent(String name, SettingsValue value) {
        return false ;
    }

    @Override
    public void run(double dt) {
        double power, delta;

        power = first_.getMotorOutputLeadVoltage();
        delta = power * first_ticks_per_second_per_volt_ * dt;
        first_angle_ += delta ;
        first_.setIntegratedSensorRawPosition((int)first_angle_) ;

        power = second_.getMotorOutputLeadVoltage();
        delta = power * second_ticks_per_second_per_volt_ * dt;
        second_angle_ += delta ;
        second_.setIntegratedSensorRawPosition((int)first_angle_) ;
    }
}
