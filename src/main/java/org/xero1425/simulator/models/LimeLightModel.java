package org.xero1425.simulator.models;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

public class LimeLightModel extends SimulationModel {
    public LimeLightModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

        table_ = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean create() {

        if (!hasProperty("latency")) {
            latency_ = 0.01 ;
        }
        else {
            SettingsValue prop = getProperty("latency") ;
            if (!prop.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger() ;
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" property ").addQuoted("latency").add(" is not a double") ;
                logger.endMessage();
            }
            else {
                try {
                    latency_ = prop.getDouble();
                } catch (BadParameterTypeException e) {
                }
            }
        }

        setCreated();
        return true;
    }

    public void run(double dt) {
        table_.getEntry("tl").setNumber(latency_) ;
    }

    public void setTX(double v) {
        table_.getEntry("tx").setNumber(v) ;        
    }

    public void setTV(double v) {
        table_.getEntry("tv").setNumber(v) ;        
    }

    public void setTY(double v) {
        table_.getEntry("ty").setNumber(v) ;        
    }    

    public boolean processEvent(String name, SettingsValue value) {
        boolean ret = false;

        if (name.equals("tv") || name.equals("ty") || name.equals("tx")) {
            ret = true ;
            
            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();
            }
            try {
                table_.getEntry(name).setNumber(value.getDouble());
            } catch (BadParameterTypeException e) {
            }
        }

        return ret ;
    }

    private NetworkTable table_ ;
    private double latency_ ;
}