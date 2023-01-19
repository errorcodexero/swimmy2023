package org.xero1425.base.pneumatics;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class XeroDoubleSolenoid extends XeroSolenoidBase {
    private DoubleSolenoid solenoid_ ;

    private SimInt simstate_ ;
    private DoubleSolenoid.Value state_ ;

    static final public String SimDeviceName = "SimSolenoid" ;
    static final public String SimeStateNamePrefix = "double" ;

    public XeroDoubleSolenoid(XeroRobot robot, int module, int forward, int reverse) {
        super(robot) ;
        if (XeroRobot.isSimulation()) {
            String name = getSimulatedName(module, forward, reverse) ;
            simstate_ = getSimulatedDevice().createInt(name, SimDevice.Direction.kBidir, 0) ;
        }
        else {
            solenoid_ = new DoubleSolenoid(module, robot.getPneumaticsType(), forward, reverse) ;
        }

        state_ = DoubleSolenoid.Value.kOff ;
    }

    public XeroDoubleSolenoid(XeroRobot robot, int forward, int reverse) {
        super(robot) ;

        if (XeroRobot.isSimulation()) {
            String name = getSimulatedName(0, forward, reverse) ;
            simstate_ = getSimulatedDevice().createInt(name, SimDevice.Direction.kBidir, 0) ;
        }
        else {
            solenoid_ = new DoubleSolenoid(robot.getPneumaticsType(), forward, reverse) ;
        }
    }

    public XeroDoubleSolenoid(Subsystem sub, String name) throws BadParameterTypeException, MissingParameterException {
        this(sub.getRobot(), sub.getSettingsValue("hw:solenoids:" + name + ":forward").getInteger(),
        sub.getSettingsValue("hw:solenoids:" + name + ":reverse").getInteger()) ; 
    }

    public XeroDoubleSolenoid(Subsystem sub, int module, String name) throws BadParameterTypeException, MissingParameterException {
        this(sub.getRobot(), module, 
                sub.getSettingsValue("hw:solenoids:" + name + ":forward").getInteger(),
                sub.getSettingsValue("hw:solenoids:" + name + ":reverse").getInteger()) ; 
    }

    public void close() {
        if (!XeroRobot.isSimulation()) {
            solenoid_.close() ;
        }
    }

    public void set(DoubleSolenoid.Value value) {
        state_ = value ;

        if (XeroRobot.isSimulation()) {
            simstate_.set(value.ordinal());
        }
        else {
            solenoid_.set(value) ;
        }
    }

    public DoubleSolenoid.Value get() {
        return state_ ;
    }

    public void toggle() {
        if (state_ == DoubleSolenoid.Value.kForward) {
            set(DoubleSolenoid.Value.kReverse) ;
        }
        else if (state_ == DoubleSolenoid.Value.kReverse) {
            set(DoubleSolenoid.Value.kForward) ;
        }
    }

    public boolean isFwdSolenoidDisabled() {
        boolean ret = false ;

        if (!XeroRobot.isSimulation()) {
            ret = solenoid_.isFwdSolenoidDisabled() ;
        }

        return ret ;
    }

    public boolean isRevSolenoidDisabled() {
        boolean ret = false ;

        if (!XeroRobot.isSimulation()) {
            ret = solenoid_.isRevSolenoidDisabled() ;
        }

        return ret ;
    }

    static public String getSimulatedName(int module, int forward, int reverse) {
        return SimeStateNamePrefix + "-" + module + "-" + forward + "-" + reverse ;
    }
}
