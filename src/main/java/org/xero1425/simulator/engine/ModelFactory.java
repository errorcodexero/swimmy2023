package org.xero1425.simulator.engine;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;
import java.util.Map;

import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

public class ModelFactory {
    public ModelFactory(SimulationEngine engine) {
        engine_ = engine ;
        name_to_class_ = new HashMap<String, String>();
    }

    public void registerModel(String name, String cname) {
        name_to_class_.put(name, cname);
    }

    public SimulationModel createModel(String name, String inst)
            throws ClassNotFoundException, NoSuchMethodException, SecurityException, InstantiationException,
            IllegalAccessException, IllegalArgumentException, InvocationTargetException {
                
        if (!name_to_class_.containsKey(name)) {
            MessageLogger logger = engine_.getMessageLogger();
            logger.startMessage(MessageType.Warning) ;
            logger.add("cannot create simulation model for model name ").addQuoted(name) ;
            logger.add(" instance ").addQuoted(inst).add(" - no such model was registered") ;
            logger.endMessage();
            return null ;
        }

        String clname = name_to_class_.get(name) ;
        Class<?> clazz = Class.forName(clname) ;
        Constructor<?> ctor = clazz.getConstructor(SimulationEngine.class, String.class, String.class) ;
        Object obj = ctor.newInstance(engine_, name, inst) ;
        return (SimulationModel)obj ;
    }

    private Map<String, String> name_to_class_ ;
    private SimulationEngine engine_ ;
}