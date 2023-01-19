package org.xero1425.simulator.models;

import org.xero1425.simulator.engine.SimulationModel;
import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.misc.SettingsValue;

public class NavXModel extends SimulationModel {
    private final static String YawDeviceName = "navX-Sensor[0]" ;
    private final static String YawValueName = "Yaw" ;
    
    private int sim_dev_handle_ ;
    private int sim_dev_yaw_handle_ ;

    public NavXModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst) ;
    }

    public void deviceCreated(String name, int handle) {
    }

    public boolean create() {
        sim_dev_handle_ = SimDeviceDataJNI.getSimDeviceHandle(YawDeviceName) ;
        sim_dev_yaw_handle_ = SimDeviceDataJNI.getSimValueHandle(sim_dev_handle_, YawValueName) ;
        setYaw(0.0) ;

        setCreated();
        return true ;
    }

    public void run(double dt) {
    }

    public boolean processEvent(String name, SettingsValue value) {
        return true ;
    }

    public void setTotalAngle(double v) {
    }

    public void setYaw(double v) {
        SimDeviceJNI.setSimValueDouble(sim_dev_yaw_handle_, v);
    }

    public double getYaw() {
        return SimDeviceJNI.getSimValueDouble(sim_dev_yaw_handle_) ;
    }

}