package frc.models;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.models.SimMotorController;

public class ArmModel extends SimulationModel {
    private SimMotorController first_ ;
    private double first_ticks_per_second_per_volt_ ;
    private double first_angle_ ;

    private SimMotorController second_ ;
    private double second_ticks_per_second_per_volt_ ;
    private double second_angle_ ;

    public ArmModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }

    @Override
    public boolean create() {
        first_ = new SimMotorController(this, "first") ;
        if (!first_.createMotor()) {
            return false ;
        }

        try {
            first_ticks_per_second_per_volt_ = getProperty("first:ticks_per_second_per_volt").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(getModelName()).add(" instance ").addQuoted(getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("first:ticks_per_second_per_volt").endMessage();
            return false ;
        }

        second_ = new SimMotorController(this, "second") ;
        if (!second_.createMotor())
            return false ;

        try {
            second_ticks_per_second_per_volt_ = getProperty("second:ticks_per_second_per_volt").getDouble();
        } catch (BadParameterTypeException e) {
            MessageLogger logger = getEngine().getMessageLogger() ;
            logger.startMessage(MessageType.Error) ;
            logger.add("cannot create model ").addQuoted(getModelName()).add(" instance ").addQuoted(getInstanceName()) ;
            logger.add(" - missing parameter ").addQuoted("second:ticks_per_second_per_volt").endMessage();
            return false ;
        }

        setCreated(); 
        return true ;
    }

    @Override
    public boolean processEvent(String name, SettingsValue value) {
        return false ;
    }

    @Override
    public void run(double dt) {
        double power, delta;

        power = first_.getPower() ;
        delta = power * first_ticks_per_second_per_volt_ * dt;
        first_angle_ += delta ;
        first_.setEncoder(first_angle_);

        power = second_.getPower() ;
        delta = power * second_ticks_per_second_per_volt_ * dt;
        second_angle_ += delta ;
        second_.setEncoder(second_angle_);
    }
}
