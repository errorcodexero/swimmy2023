package org.xero1425.simulator.engine;

import java.util.HashMap;
import java.util.Map;

import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

public abstract class SimulationModel {
    
    private SimulationEngine engine_ ;
    private String model_ ;
    private String instance_ ;
    private Map<String, SettingsValue> props_ ;
    private boolean created_ ;
    private int logger_id_ ;
    private boolean warned_not_run_ ;
    
    public SimulationModel(SimulationEngine engine, String model, String instance) {
        engine_ = engine ;
        model_ = model ;
        instance_ = instance ;
        created_ = false ;
        warned_not_run_ = false ;

        props_ = new HashMap<String, SettingsValue>() ;

        logger_id_ = engine.getMessageLogger().registerSubsystem(model + "_model") ;
    }

    public boolean warnedNotRun() {
        return warned_not_run_ ;
    }

    public void setWarnedNotRun() {
        warned_not_run_ = true ;
    }

    public String statusString() {
        return "" ;
    }

    public String getModelName() {
        return model_ ;
    }

    public String getInstanceName() {
        return instance_ ;
    }

    public abstract boolean create() ;
    public abstract void run(double dt) ;
    public abstract boolean processEvent(String name, SettingsValue value) ;
    public void startCycle()  {
    }

    public void endCycle() {
    }

    public boolean hasProperty(String name) {
        return props_.containsKey(name) ;
    }

    public void setProperty(String name, SettingsValue value) {
        props_.put(name, value) ;
    }

    public SettingsValue getProperty(String name) {
        return props_.get(name) ;
    }

    public double getRobotTime() {
        return engine_.getRobot().getTime() ;
    }

    public SimulationEngine getEngine() {
        return engine_ ;
    }
    
    public boolean isCreated() {
        return created_ ;
    }

    protected void setCreated() {
        created_ = true ;
    }

    protected int getLoggerID() {
        return logger_id_ ;
    }
    
    protected int getIntProperty(String name) throws Exception {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (!hasProperty(name)) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" is missing required property").addQuoted(name);
            logger.endMessage();
            throw new Exception("getIntProperty failed") ;
        }

        SettingsValue value = getProperty(name) ;
        if (!value.isInteger()) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" property ").addQuoted(name).add(" is not an integer");
            logger.endMessage();   
            throw new Exception("getIntProperty failed") ;         
        }

        return value.getInteger() ;
    }

    protected double getDoubleProperty(String name) throws Exception {
        double ret = Double.NaN ;
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (!hasProperty(name)) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" is missing required property").addQuoted(name);
            logger.endMessage();
            throw new Exception("getDoubleProperty failed") ;
        }

        SettingsValue value = getProperty(name) ;
        if (!value.isDouble() && !value.isInteger()) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" property ").addQuoted(name).add(" is not an integer");
            logger.endMessage();   
            throw new Exception("getDoubleProperty failed") ;         
        }
        else if (value.isDouble()) {
            ret = value.getDouble() ;
        }
        else if (value.isInteger()) {
            ret = value.getInteger() ;
        }

        return ret ;
    }

    protected String getStringProperty(String name) throws Exception {
        MessageLogger logger = getEngine().getMessageLogger() ;

        if (!hasProperty(name)) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" is missing required property").addQuoted(name);
            logger.endMessage();
            throw new Exception("getStringProperty failed") ;
        }

        SettingsValue value = getProperty(name) ;
        if (!value.isString()) {
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" property ").addQuoted(name).add(" is not a string");
            logger.endMessage();   
            throw new Exception("getStringProperty failed") ;         
        }

        return value.getString() ;
    }    

}