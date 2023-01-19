/// \file EventsManager.java
/// This class manages a timeline of events for a simulation.  The
/// events are read from a file and are sent to the simulation when
/// the time associated with the event is reached.
///

package org.xero1425.simulator.engine;

import java.util.ArrayList;
import java.util.List;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.xero1425.misc.JsonReader;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

public class EventsManager {
    // The simulation engine that owns this event manager
    private SimulationEngine engine_ ;

    // The set of events read from an events file to be applied to a simulation
    private List<SimulationEvent> events_ ;
    
    /// \brief create the events manager
    /// \param engine the simulation engine
    public EventsManager(SimulationEngine engine) {
        engine_ = engine ;
        events_ = new ArrayList<SimulationEvent>() ;
    }

    public boolean readEventsFile(String file) {
        MessageLogger logger = engine_.getMessageLogger();

        logger.startMessage(MessageType.Info);
        logger.add("reading simulator events file ").addQuoted(file) ;
        logger.endMessage(); 

        JSONObject jobj = JsonReader.readFile(file, logger) ;
        if (jobj == null) {
            logger.startMessage(MessageType.Error);
            logger.add("cannot read events file ").addQuoted(file).add(" - ");
            logger.add("error occurred while reading file") ;
            return false;
        }

        Object obj ;

        if (jobj.containsKey("purpose")) {
            obj = jobj.get("purpose") ;
            if (obj instanceof JSONArray) {
                JSONArray a = (JSONArray)obj ;
                logger.startMessage(MessageType.Info).add("  ").endMessage();
                for(int i = 0 ; i < a.size() ; i++) {
                    Object o = a.get(i) ;
                    if (o instanceof String) {
                        logger.startMessage(MessageType.Info) ;
                        logger.add((String)o) ;
                        logger.endMessage();                        
                    }
                }
                logger.startMessage(MessageType.Info).add("  ").endMessage();           
            }
            else if (obj instanceof String) {
                logger.startMessage(MessageType.Info).add("  ").endMessage();           
                logger.startMessage(MessageType.Info) ;
                logger.add((String)obj) ;
                logger.endMessage();
                logger.startMessage(MessageType.Info).add("  ").endMessage();              
            }
        }
        obj = jobj.get("stimulus");

        if (!(obj instanceof JSONArray)) {
            logger.startMessage(MessageType.Error);
            logger.add("cannot read events file ").addQuoted(file).add(" - ");
            logger.add("top level json object does not contains a models entry");
            return false;
        }

        JSONArray stimarray = (JSONArray) obj;

        for (int i = 0; i < stimarray.size(); i++) {
            obj = stimarray.get(i);
            if (obj instanceof JSONObject) {
                try {
                    parseTimePoint((JSONObject) obj);
                }
                catch(Exception ex) {
                    logger.startMessage(MessageType.Error);
                    logger.add("cannot read events file ").addQuoted(file).add(" - ");
                    logger.add("error parsing time point - ");
                    logger.add(ex.getMessage()) ;
                    logger.endMessage();
                    return false;                    
                }
            }
        }

        return true;        
    }

    public int size() {
        return events_.size() ;
    }

    public SimulationEvent getFirstEvent() {
        return events_.get(0) ;
    }

    public void removeFirstEvent() {
        events_.remove(0) ;
    }

    private void parseTimePoint(JSONObject tpt) throws Exception {
        Object obj ;

        if (!tpt.containsKey("time"))
            return ;

        obj = tpt.get("time") ;
        if (!(obj instanceof Double))
            return ;

        double t = (Double)obj ;

        if (tpt.containsKey("events")) {
            obj = tpt.get("events") ;
            if (obj instanceof JSONArray)
                parseSimEvents(t, (JSONArray)obj) ;
        }

        if (tpt.containsKey("asserts")) {
            obj = tpt.get("asserts") ;
            if (obj instanceof JSONArray)
                parseSimAsserts(t, (JSONArray)obj);
        }
    }

    private void parseSimEvents(double t, JSONArray evs) {
        MessageLogger logger = engine_.getMessageLogger() ;

        for(int i = 0 ; i < evs.size() ; i++) {
            Object obj = evs.get(i) ;
            if (!(obj instanceof JSONObject))
                continue ;

            JSONObject jobj = (JSONObject)obj ;

            if (!jobj.containsKey("model")) {
                logger.startMessage(MessageType.Warning) ;
                logger.add("events at index ").add(i).add(" is missing the 'model' property") ;
                logger.endMessage();
                continue ;
            }

            if (!jobj.containsKey("instance")) {
                logger.startMessage(MessageType.Warning) ;
                logger.add("events at index ").add(i).add(" is missing the 'instance' property") ;
                logger.endMessage();                
                continue ;
            }

            if (!jobj.containsKey("values")) {
                logger.startMessage(MessageType.Warning) ;
                logger.add("events at index ").add(i).add(" is missing the 'values' property") ;
                logger.endMessage();                
                continue ;
            }            

            Object mobj = jobj.get("model") ;
            Object iobj = jobj.get("instance") ;
            Object vobj = jobj.get("values") ;

            if (!(mobj instanceof String)) {
                logger.startMessage(MessageType.Warning) ;
                logger.add("events at index ").add(i).add(" has 'model' property but it is not a string") ;
                logger.endMessage();
                continue ;
            }
            
            if (!(iobj instanceof String)) {
                logger.startMessage(MessageType.Warning) ;
                logger.add("events at index ").add(i).add(" has 'instance' property but it is not a string") ;
                logger.endMessage();
                continue ;                
            }

            if (!(vobj instanceof JSONObject)) {
                logger.startMessage(MessageType.Warning) ;
                logger.add("events at index ").add(i).add(" has 'values' property but it is not a JSON object") ;
                logger.endMessage();
                continue ;                  
            }

            SimulationModel model = engine_.findModel((String)mobj, (String)iobj) ;
            if (model == null) {
                logger.startMessage(MessageType.Warning) ;
                logger.add("events at index ").add(i).add(" does not have simulation model with model ") ;
                logger.addQuoted((String)mobj).add(" and instance ").addQuoted((String)iobj) ;
                logger.endMessage();

                return ;
            }

            JSONObject vjobj = (JSONObject)vobj ;
            for(Object key : vjobj.keySet()) {
                String keystr = (String)key ;
                Object evval = vjobj.get(key) ;
                SettingsValue v = null ;

                if (evval instanceof String) {
                    v = new SettingsValue((String)evval) ;
                }
                else if (evval instanceof Integer) {
                    v = new SettingsValue((Integer)evval) ;
                }
                else if (evval instanceof Long) {
                    v = new SettingsValue((Long)evval) ;
                }
                else if (evval instanceof Boolean) {
                    v = new SettingsValue((Boolean)evval) ;                        
                }
                else if (evval instanceof Double) {
                    v = new SettingsValue((Double)evval) ;  
                }
                else {
                    v = null ;
                }

                if (v != null) {
                    SimulationModelEvent ev = new SimulationModelEvent(t, model.getModelName(), model.getInstanceName(), keystr, v) ;
                    insertEvent(ev) ;
                }
            }
        }
    }

    private void parseSimAsserts(double t, JSONArray evs) throws Exception {
        MessageLogger logger = engine_.getMessageLogger() ;

        for(int i = 0 ; i < evs.size() ; i++) {
            Object obj = evs.get(i) ;
            if (!(obj instanceof JSONObject))
            {
                logger.startMessage(MessageType.Warning) ;
                logger.add("in simulation file, at time", t).add(", events at index ").add(i).add(" is not a JSON object") ;
                logger.endMessage();
                continue ;
            }

            JSONObject jobj = (JSONObject)obj ;

            if (!jobj.containsKey("subsystem")) {
                logger.startMessage(MessageType.Warning) ;
                logger.add("events at index ").add(i).add(" is missing the 'model' property") ;
                logger.endMessage();
                continue ;
            }

            if (!jobj.containsKey("property")) {
                logger.startMessage(MessageType.Warning) ;
                logger.add("events at index ").add(i).add(" is missing the 'instance' property") ;
                logger.endMessage();                
                continue ;
            }

            if (!jobj.containsKey("value") && !jobj.containsKey("setting")) {
                logger.startMessage(MessageType.Warning) ;
                logger.add("events at index ").add(i).add(" is missing the 'value' or the 'setting' property") ;
                logger.endMessage();                
                continue ;
            }            

            Object mobj = jobj.get("subsystem") ;
            Object iobj = jobj.get("property") ;

            if (!(mobj instanceof String)) {
                logger.startMessage(MessageType.Warning) ;
                logger.add("events at index ").add(i).add(" has 'model' property but it is not a string") ;
                logger.endMessage();
                continue ;
            }
            
            if (!(iobj instanceof String)) {
                logger.startMessage(MessageType.Warning) ;
                logger.add("events at index ").add(i).add(" has 'instance' property but it is not a string") ;
                logger.endMessage();
                continue ;                
            }

            double tolerance = 1e-9 ;

            if (jobj.containsKey("tolerance")) {
                Object tobj = jobj.get("tolerance") ;
                if (tobj instanceof Double) {
                    tolerance = (Double)tobj ;
                }
                else if (tobj instanceof Long) {
                    tolerance = (Long)tobj ;
                }
            }

            if (jobj.containsKey("value")) {
                Object vobj = jobj.get("value") ;
                SettingsValue v = null ;

                if (vobj instanceof String) {
                    v = new SettingsValue((String)vobj) ;
                }
                else if (vobj instanceof Integer) {
                    v = new SettingsValue((Integer)vobj) ;
                }
                else if (vobj instanceof Long) {
                    v = new SettingsValue((Long)vobj) ;
                }            
                else if (vobj instanceof Boolean) {
                    v = new SettingsValue((Boolean)vobj) ;                        
                }
                else if (vobj instanceof Double) {
                    v = new SettingsValue((Double)vobj) ;
                    if (jobj.containsKey("tolerance")) {
                        Object dobj = jobj.get("tolerance") ;
                        if (!(dobj instanceof Double)) {
                            logger.startMessage(MessageType.Warning) ;
                            logger.add("events at index ").add(i).add(" has 'tolerance' property but it is not a double - tolerance defaults to 1e-9") ;
                            logger.endMessage();                        
                        }
                        else {
                            tolerance = (Double)dobj ;
                        }
                    }
                }
                else {
                    v = null ;
                }

                if (v != null) {
                    SimulationAssertEvent ev = new SimulationAssertEvent(t, (String)mobj, (String)iobj, v, tolerance) ;
                    if (v.isDouble())
                        ev.setTolerance(tolerance) ;
                    insertEvent(ev) ;
                }
            }
            else {
                Object vobj = jobj.get("setting") ;
                if (!(vobj instanceof String)) {
                    logger.startMessage(MessageType.Warning) ;
                    logger.add("events at index ").add(i).add(" has 'setting' property but it is not a string") ;
                    logger.endMessage();
                    continue ;                
                }

                SimulationAssertEvent ev = new SimulationAssertEvent(t, (String)mobj, (String)iobj, (String)vobj, tolerance) ;
                insertEvent(ev) ;
            }
        }
    }

    private void insertEvent(SimulationEvent ev) {
        if (events_.size() == 0) {
            events_.add(ev) ;
        } else {
            boolean ins = false ;
            int i = 0 ;
            while (i < events_.size()) {
                if (ev.getTime() < events_.get(i).getTime()) {
                    events_.add(i, ev) ;
                    ins = true ;
                    break ;
                }

                i++ ;
            }

            if (!ins)
                events_.add(ev) ;
        }
    }
} ;
