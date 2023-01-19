package org.xero1425.simulator.models;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

public class DriverGamepadModel extends SimulationModel {
    private static final String buttonEvent = "button";
    private static final String axisEvent = "axis";
    private static final String povEvent = "pov";

    public DriverGamepadModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);
    }

    public boolean create() {
        int count = 0;

        if (hasProperty("index")) {
            SettingsValue v = getProperty("index");
            if (!v.isInteger()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" has property ").addQuoted("index");
                logger.add(" but its not an integer");
                logger.endMessage();
                return false;
            }

            try {
                index_ = v.getInteger();
            } catch (BadParameterTypeException e) {
            }
        } else {
            MessageLogger logger = getEngine().getMessageLogger();
            logger.startMessage(MessageType.Error);
            logger.add("event: model ").addQuoted(getModelName());
            logger.add(" instance ").addQuoted(getInstanceName());
            logger.add(" is missing required property").addQuoted("index");
            logger.endMessage();
            return false;
        }

        if (hasProperty("axes")) {
            SettingsValue v = getProperty("axes");
            if (!v.isInteger()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" has property ").addQuoted("axes");
                logger.add(" but its not an integer");
                logger.endMessage();
                return false;
            }

            try {
                count = v.getInteger();
            } catch (BadParameterTypeException e) {
            }

            axes_ = new float[count];

            for (int i = 0; i < count; i++)
                axes_[i] = 0.0f;

            DriverStationDataJNI.setJoystickAxes((byte) index_, axes_);
        }

        if (hasProperty("buttons")) {
            SettingsValue v = getProperty("buttons");
            if (!v.isInteger()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" has property ").addQuoted("buttons");
                logger.add(" but its not an integer");
                logger.endMessage();
                return false;
            }

            try {
                button_count_ = v.getInteger();
            } catch (BadParameterTypeException e) {
            }

            buttons_ = 0;
            DriverStationDataJNI.setJoystickButtons((byte) index_, buttons_, button_count_);
        }

        if (hasProperty("povs")) {
            SettingsValue v = getProperty("povs");
            if (!v.isInteger()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" has property ").addQuoted("povs");
                logger.add(" but its not an integer");
                logger.endMessage();
                return false;
            }

            try {
                count = v.getInteger();
            } catch (BadParameterTypeException e) {
            }

            povs_ = new short[count];

            for (int i = 0; i < count; i++)
                povs_[i] = -1 ;

            DriverStationDataJNI.setJoystickPOVs((byte) index_, povs_);
        }


        DriverStationSim.notifyNewData();

        setCreated();
        return true ;
    }

    public void run(double dt) {
    }

    public boolean processEvent(String name, SettingsValue value) {
        int which = 0 ;

        if (name.startsWith(buttonEvent)) {
            try {
                which = Integer.parseInt(name.substring(buttonEvent.length())) ;
            }
            catch(NumberFormatException ex) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" is not valid, should be button followed by an integer (e.g. button2)").endMessage();    
                return true ;            
            }

            if (which > button_count_) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" exceeds the defined max button count").endMessage();

                return true ;
            }

            if (!value.isBoolean()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a boolean").endMessage();                
            }

            try {
                if (value.getBoolean())
                    buttons_ |= (1 << (which - 1)) ;
                else
                    buttons_ &= ~(1 << (which - 1)) ;
            }
            catch(BadParameterTypeException e) {
            }

            DriverStationDataJNI.setJoystickButtons((byte) index_, buttons_, button_count_);
        }
        else if (name.startsWith(axisEvent)) {
            try {
                which = Integer.parseInt(name.substring(axisEvent.length())) ;
            }
            catch(NumberFormatException ex) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" is not valid, should be 'axis' followed by an integer (e.g. axis9)").endMessage();                
            }

            if (which > axes_.length) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" exceeds the defined max axis count").endMessage();

                return true ;
            }

            if (!value.isDouble()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not a double").endMessage();                
            }            

            try {
                axes_[which] = (float)value.getDouble() ;
            }
            catch(BadParameterTypeException e) {
            }

            DriverStationDataJNI.setJoystickAxes((byte) index_, axes_) ;
        }
        else if (name.startsWith(povEvent)) {
            try {
                which = Integer.parseInt(name.substring(povEvent.length())) ;
            }
            catch(NumberFormatException ex) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" is not valid, should be 'pov' followed by an integer (e.g. pov0)").endMessage();                
            }

            if (which > povs_.length) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" exceeds the defined max pov count").endMessage();

                return true ;
            }

            if (!value.isInteger()) {
                MessageLogger logger = getEngine().getMessageLogger();
                logger.startMessage(MessageType.Error);
                logger.add("event: model ").addQuoted(getModelName());
                logger.add(" instance ").addQuoted(getInstanceName());
                logger.add(" event name ").addQuoted(name);
                logger.add(" value is not an integer").endMessage();                
            }            

            try {
                povs_[which] = (short)value.getInteger() ;
            }
            catch(BadParameterTypeException e) {
            }

            DriverStationDataJNI.setJoystickPOVs((byte) index_, povs_) ;
        }        
        return true ;
    }

    private int index_ ;

    private int buttons_ ;
    private int button_count_ ;

    private float[] axes_ ;

    private short[] povs_ ;
}