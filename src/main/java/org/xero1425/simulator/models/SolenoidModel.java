package org.xero1425.simulator.models;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.xero1425.base.pneumatics.XeroDoubleSolenoid;
import org.xero1425.base.pneumatics.XeroSolenoid;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class SolenoidModel {
    private static SolenoidModel theone_ = null ;

    private MessageLogger logger_ ;
    private int dev_handle_ ;
    private Map<Integer, List<Integer>> channels_ ;

    private SolenoidModel(MessageLogger logger) {
        logger_ = logger ;
        dev_handle_ = SimDeviceDataJNI.getSimDeviceHandle(XeroDoubleSolenoid.SimDeviceName) ;
        channels_ = new HashMap<Integer, List<Integer>>() ;
    }

    public static SolenoidModel getInstance(MessageLogger logger) {
        if (theone_ == null)
            theone_ = new SolenoidModel(logger) ;

        return theone_ ;
    }

    public int getSolenoid(int module, int channel) {
        List<Integer> list ;

        if (!channels_.containsKey(module)) {
            list = new ArrayList<Integer>() ;
            channels_.put(module, list) ;
        } else {
            list = channels_.get(module) ;
        }

        if (list.contains(channel)) {
            logger_.startMessage(MessageType.Error) ;
            logger_.add(": cannot create single solenoid model on channel ").add(channel).add(", it is already in use").endMessage();
        }

        String name = XeroSolenoid.getSimulatedName(module, channel) ;        
        int vhandle = SimDeviceDataJNI.getSimValueHandle(dev_handle_, name) ;
        return vhandle ;
    }

    public int getDoubleSolenoid(int module, int forward, int reverse) {
        List<Integer> list ;

        if (!channels_.containsKey(module)) {
            list = new ArrayList<Integer>() ;
            channels_.put(module, list) ;
        } else {
            list = channels_.get(module) ;
        }

        if (list.contains(forward)) {
            logger_.startMessage(MessageType.Error) ;
            logger_.add(": cannot create double solenoid model on forward channel ").add(forward).add(", it is already in use").endMessage();
        }

        if (list.contains(reverse)) {
            logger_.startMessage(MessageType.Error) ;
            logger_.add(": cannot create double solenoid model on forward channel ").add(reverse).add(", it is already in use").endMessage();
        }

        String name = XeroDoubleSolenoid.getSimulatedName(module, forward, reverse) ;
        int vhandle = SimDeviceDataJNI.getSimValueHandle(dev_handle_, name) ;
        return vhandle ;
    }

    public DoubleSolenoid.Value getDoubleSolenoidState(int handle) {
        HALValue v = SimDeviceJNI.getSimValue(handle) ;
        DoubleSolenoid.Value value = DoubleSolenoid.Value.values()[(int)(v.getLong())] ;
        return value ;
    }

    public boolean getSingleSolenoidState(int handle) {
        HALValue v = SimDeviceJNI.getSimValue(handle) ;
        return v.getBoolean() ;
    }
}
