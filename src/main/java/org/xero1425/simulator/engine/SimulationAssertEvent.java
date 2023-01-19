package org.xero1425.simulator.engine;

import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsValue;

public class SimulationAssertEvent extends SimulationEvent {

    private String subsystem_;
    private String name_;
    private String setting_;
    private SettingsValue value_;
    private double tolerance_;

    public SimulationAssertEvent(double t, String subsystem, String name, SettingsValue v, double tol) {
        super(t);

        subsystem_ = subsystem;
        name_ = name;
        value_ = v;
        setting_ = null;

        tolerance_ = tol ;
    }

    public SimulationAssertEvent(double t, String subsystem, String name, String setting, double tol) {
        super(t);
        subsystem_ = subsystem;
        name_ = name;
        value_ = null;
        setting_ = setting;

        tolerance_ = tol ;
    }

    public void run(SimulationEngine engine) {
        Subsystem sub = engine.getRobot().getRobotSubsystem().getSubsystemByName(subsystem_);
        if (sub == null) {
            MessageLogger logger = engine.getMessageLogger();
            logger.startMessage(MessageType.Error);
            logger.add("AssertFailed: ");
            logger.add("subsystem ", subsystem_);
            logger.add(" - does not exist in the robot");
            logger.endMessage();
            engine.addAssertError();
        } else {
            MessageLogger logger = engine.getMessageLogger();
            SettingsValue v = sub.getProperty(name_);
            if (v == null) {
                logger.startMessage(MessageType.Error);
                logger.add("AssertFailed: ");
                logger.add("subsystem ", subsystem_);
                logger.add(" property ", name_);
                logger.add(" - subsystem did not contain the given property");
                logger.endMessage();
                engine.addAssertError();
            } else {
                boolean pass = false;

                SettingsValue value = getValue(engine) ;
                if (value == null) {
                    logger.startMessage(MessageType.Error);
                    logger.add("AssertFailed: ");
                    logger.add("subsystem ", subsystem_);
                    logger.add(" property ", name_);
                    logger.add(" - the params file did not contain the property").addQuoted(setting_) ;
                    logger.endMessage();
                    engine.addAssertError();
                }

                if (v.isDouble()) {
                    try {
                        pass = Math.abs(v.getDouble() - value.getDouble()) < tolerance_;
                    } catch (BadParameterTypeException e) {
                        // Should never happen
                        pass = false;
                    }
                } else {
                    pass = v.equals(value);
                }

                if (!pass) {
                    logger.startMessage(MessageType.Error);
                    logger.add("AssertFailed: ");
                    logger.add("subsystem ").addQuoted(subsystem_) ;
                    logger.add(", property ").addQuoted(name_) ;
                    logger.add(", expected ").addQuoted(value.toString());
                    logger.add(", got ").addQuoted(v.toString());
                    logger.endMessage();
                    engine.addAssertError();
                } else {
                    logger.startMessage(MessageType.Info);
                    logger.add("AssertPassed: ");
                    logger.add("subsystem", subsystem_);
                    logger.add(", property ").addQuoted(name_) ;
                    logger.add(", value ").addQuoted(value.toString());
                    logger.endMessage();
                    engine.addAssertPassed();
                }
            }
        }
    }

    public String toString() {
        return "SimulationAssertEvent";
    }

    public void setTolerance(double v) {
        tolerance_ = v;
    }

    private SettingsValue getValue(SimulationEngine engine) {
        SettingsValue ret = null;

        if (value_ != null) {
            ret = value_;
        } else {
            try {
                ret = engine.getRobot().getSettingsSupplier().get(setting_);
            } catch (MissingParameterException e) {
                ret = null ;
            }
        }

        return ret ;
    }

}