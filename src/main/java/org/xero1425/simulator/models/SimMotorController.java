package org.xero1425.simulator.models;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.base.motors.CTREMotorController;
import org.xero1425.base.motors.MotorController;
import org.xero1425.base.motors.SparkMaxMotorController;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.SettingsValue;

public class SimMotorController {
    
    String name_ ;
    int handle_ ;
    private SimulationModel model_ ;
    private int count_ ;
    private int index_ ;
    private int ticks_per_rev_ ;
    
    public SimMotorController(SimulationModel model, String name) {
        model_ = model;
        name_ = name ;
        ticks_per_rev_ = -1 ;
    }

    public int ticksPerRev() {
        return ticks_per_rev_ ;
    }

    public int getIndex() {
        return index_ ;
    }

    public int getCount() {
        return count_ ;
    }

    public double getPower() {
        int vhandle = SimDeviceDataJNI.getSimValueHandle(handle_, MotorController.SimPowerParamName) ;
        HALValue v = SimDeviceJNI.getSimValue(vhandle) ;
        if (v.getType() != HALValue.kDouble)
            return 0.0 ;

        return v.getDouble() ;
    }

    public void setEncoder(double v) {
        int vhandle = SimDeviceDataJNI.getSimValueHandle(handle_, MotorController.SimEncoderParamName) ;
        SimDeviceJNI.setSimValueDouble(vhandle, v);
    }

    public boolean usesTicks() {
        int vhandle = SimDeviceDataJNI.getSimValueHandle(handle_, MotorController.SimEncoderStoresTicksParamName) ;
        return SimDeviceJNI.getSimValue(vhandle).getBoolean() ;
    }

    public boolean createMotor() {
        handle_ = -1 ;
        if (createSingleMotor(name_ + ":motor")) {
            count_ = 1;
            return true;
        }

        int i = 1;
        while (true) {
            String mname = name_ + ":motor:" + i;
            if (!createSingleMotor(mname)) {
                if (i == 0)
                    return false;

                count_ = i;
                return true;
            }

            i++;
        }
    }

    private boolean createSingleMotor(String name) {

        if (!model_.hasProperty(name + ":index") || !model_.hasProperty(name + ":type"))
            return false;

        SettingsValue indexval = model_.getProperty(name + ":index");
        SettingsValue typeval = model_.getProperty(name + ":type");

        if (!indexval.isInteger())
            return false;

        if (!typeval.isString())
            return false;

        try {
            index_ = indexval.getInteger();
        } catch (BadParameterTypeException e) {
        }

        String t = null ;
        try {
            t = typeval.getString();
        } catch (BadParameterTypeException e) {
        }

        if (t.equals("talon-fx") || t.equals("talon-srx")) {
            if (handle_ == -1)
            {
                handle_ = SimDeviceDataJNI.getSimDeviceHandle(CTREMotorController.SimDeviceName + "[" + index_ + "]") ;
                ticks_per_rev_ = 2048 ;
            }
        }
        else if (t.equals("sparkmax-brushed")) {
            if (handle_ == -1)
                handle_ = SimDeviceDataJNI.getSimDeviceHandle(SparkMaxMotorController.SimDeviceNameBrushed + "[" + index_ + "]") ;
        }
        else if (t.equals("sparkmax-brushless")) {
            if (handle_ == -1)
                handle_ = SimDeviceDataJNI.getSimDeviceHandle(SparkMaxMotorController.SimDeviceNameBrushless + "[" + index_ + "]") ;            
            
                ticks_per_rev_ = 42 ;
        }        
        else {
            return false ;
        }

        if (handle_ == 0)
            return false ;
        
        return true ;        
    }
} ;