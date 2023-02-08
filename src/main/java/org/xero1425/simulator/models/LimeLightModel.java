package org.xero1425.simulator.models;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.xero1425.simulator.engine.SimulationModel;
import org.xero1425.simulator.engine.SimulationEngine;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.SettingsValue;

public class LimeLightModel extends SimulationModel {
    private NetworkTable table_ ;
    private String classifier_ ;
    private String fiducials_ ;
    private String detector_ ;
    private String retro_ ;
    private int pid_ ;
    private double tl_ ;
    private double ts_ ;
    private int v_ ;

    public LimeLightModel(SimulationEngine engine, String model, String inst) {
        super(engine, model, inst);

        table_ = NetworkTableInstance.getDefault().getTable("limelight");
        classifier_ = "[]" ;
        fiducials_ = "[]" ;
        detector_ = "[]" ;
        retro_ = "[]" ;
    }

    public boolean create() {
        setCreated();
        return true;
    }

    public boolean processEvent(String name, SettingsValue value) {
        boolean ret = false;

        if (name.equals("Fiducial")) {
            try {
                fiducials_ = value.getString() ;
            } catch (BadParameterTypeException e) {
                fiducials_ = "[]" ;
            }
        }
        else if (name.equals("v")) {
            try {
                v_ = value.getInteger() ;
            } catch (BadParameterTypeException e) {
                v_ = 0 ;
            }
        }
        return ret ;
    }

    @Override
    public void run(double dt) {
        NetworkTableEntry entry = table_.getEntry("json") ;
        entry.setString(jsonText());
    }

    private String jsonText() {
        String str = "{ \"Results\": {" ;
        str += "\"Fiducial\": " + fiducials_ + "," ;
        str += "\"Classifier\": " + classifier_ + "," ;
        str += "\"Detector\" :" + detector_ + "," ;
        str += "\"Retro\":" + retro_+ "," ;
        str += "\"pID\" : " + pid_ + "," ;
        str += "\"tl\" : " + tl_ + "," ;
        str += "\"ts\" : " + ts_ + "," ;
        str += "\"v\" : " + v_ ;
        str +="}}" ; 

        return str ;
    }
}