package org.xero1425.simulator.engine;

import java.lang.reflect.InvocationTargetException;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.xero1425.simulator.models.BuiltInModels;
import org.xero1425.misc.JsonReader;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;
import org.xero1425.misc.SettingsValue;

public class ModelManager {
    public ModelManager(SimulationEngine engine) {
        engine_ = engine;

        factory_ = new ModelFactory(engine) ;
        BuiltInModels.registerBuiltinModels(factory_);                
    }

    public ModelFactory getFactory() {
        return factory_ ;
    }

    public boolean readModelFile(String file) {
        MessageLogger logger = engine_.getMessageLogger();
        Object obj ;

        JSONObject jobj = JsonReader.readFile(file, logger) ;
        if (jobj == null) {
            logger.startMessage(MessageType.Error);
            logger.add("cannot read models file ").addQuoted(file).add(" - ");
            logger.add("error occurred while reading file") ;
            return false;
        }

        obj = jobj.get("models");
        if (!(obj instanceof JSONArray)) {
            logger.startMessage(MessageType.Error);
            logger.add("cannot read models file ").addQuoted(file).add(" - ");
            logger.add("top level json object does not contains a models entry");
            return false;
        }

        JSONArray modelarray = (JSONArray) obj;

        for (int i = 0; i < modelarray.size(); i++) {
            obj = modelarray.get(i);
            if (obj instanceof JSONObject) {
                parseModel((JSONObject) obj);
            }
        }

        return true;
    }

    private boolean parseModel(JSONObject jobj) {
        if (!jobj.containsKey("model"))
            return false;

        if (!jobj.containsKey("instance"))
            return false;

        Object modelname = jobj.get("model");
        if (!(modelname instanceof String))
            return false;

        Object instname = jobj.get("instance");
        if (!(instname instanceof String))
            return false;

        SimulationModel model;
        try {
            model = factory_.createModel((String) modelname, (String) instname);
        } catch (ClassNotFoundException | NoSuchMethodException | SecurityException | InstantiationException
                | IllegalAccessException | IllegalArgumentException | InvocationTargetException e) {
            model = null ;
        }

        if (model == null)
            return true ;

        if (jobj.containsKey("props")) {
            Object propobj = jobj.get("props") ;
            if (propobj instanceof JSONObject) {
                JSONObject jpropobj = (JSONObject)propobj ;
                for(Object key : jpropobj.keySet()) {
                    if (!(key instanceof String))
                        continue ;

                    String keystr = (String)key ;
                    Object vobj = jpropobj.get(key) ;
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
                    }
                    else {
                        v = null ;
                    }

                    if (v != null)
                        model.setProperty(keystr, v) ;
                }
            }
        }

        engine_.addModel(model) ;        
        return true ;
    }

    private SimulationEngine engine_ ;
    private ModelFactory factory_ ;
} ;