package org.xero1425.base.pneumatics;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.Solenoid;

public class XeroSolenoid extends XeroSolenoidBase {
    private Solenoid solenoid_ ;
    private SimBoolean simstate_ ;
    private boolean state_ ;

    final static private String SimStateName = "single" ;

    public XeroSolenoid(XeroRobot robot, int module, int channel) {
        super(robot) ;
        solenoid_ = new Solenoid(module, robot.getPneumaticsType(), channel) ;

        if (XeroRobot.isSimulation()) {
            String name = getSimulatedName(module, channel) ;
            simstate_ = getSimulatedDevice().createBoolean(name, SimDevice.Direction.kBidir, false) ;
        }
    }

    public XeroSolenoid(XeroRobot robot, int channel) {
        super(robot) ;

        if (XeroRobot.isSimulation()) {
            String name = getSimulatedName(0, channel) ;
            simstate_ = getSimulatedDevice().createBoolean(name, SimDevice.Direction.kBidir, false) ;
        }
        else {
            solenoid_ = new Solenoid(robot.getPneumaticsType(), channel) ;
        }
    }

    public XeroSolenoid(Subsystem sub, String name) throws BadParameterTypeException, MissingParameterException {
        this(sub.getRobot(), sub.getSettingsValue("hw:solenoids:" + name).getInteger()) ;
    }

    public XeroSolenoid(Subsystem sub, int module, String name)  throws BadParameterTypeException, MissingParameterException {
        this(sub.getRobot(), module, sub.getSettingsValue("hw:solenoids:" + name).getInteger()) ;
    }

    public void close() {
        if (!XeroRobot.isSimulation()) {
            solenoid_.close() ;
        }
    }

    public void set(boolean on) {
        state_ = on ;

        if (XeroRobot.isSimulation()) {
            simstate_.set(on);
        }
        else {
            solenoid_.set(on) ;
        }
    }

    public boolean get() {
        return state_ ;
    }

    public void toggle() {
        set(!state_) ;
    }

    public boolean isDisabled() {
        boolean ret = false ;

        if (!XeroRobot.isSimulation()) {
            ret = solenoid_.isDisabled() ;
        }

        return ret ;
    }

    public static String getSimulatedName(int module, int channel) {
        return SimStateName + "-" + module + "-" + channel ;
    }
}
