package org.xero1425.base.pneumatics;

import org.xero1425.base.XeroRobot;

import edu.wpi.first.hal.SimDevice;

public class XeroSolenoidBase {
    static final public String SimDeviceName = "SimSolenoid" ;

    private static SimDevice simdev_ = null ;

    private XeroRobot robot_ ;

    public XeroSolenoidBase(XeroRobot robot) {
        robot_ = robot ;

        if (simdev_ == null && XeroRobot.isSimulation())
            simdev_ = SimDevice.create(SimDeviceName) ;
    }

    protected SimDevice getSimulatedDevice() {
        return simdev_ ;
    }

    protected XeroRobot getRobot() {
        return robot_ ;
    }
}
