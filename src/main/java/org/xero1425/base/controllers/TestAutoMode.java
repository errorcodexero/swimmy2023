package org.xero1425.base.controllers ;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.ISettingsSupplier;
import org.xero1425.misc.MissingParameterException;
import org.xero1425.misc.SettingsValue;

/// \file

/// \brief This class is the base class for the standard test mode auto mode that is
/// created for each robot.  There is expected to be a derived class that is robot specific
/// that supplies the specific test modes needed.
public class TestAutoMode extends AutoMode {
    
    // The test mode to run
    private int which_ ;

    // The parameters for each test
    private Map<Integer, Map<String, SettingsValue>> parameters_ ;

    // The settings name string for the which value
    static private final String Which = "testmode:which";

    // The settings name string for the top level test mode key
    static private final String TestModeKey = "testmode" ;

    /// \brief Create a new test automode
    /// \param ctrl the automode controller that manages this mode
    /// \param name the name of the automodes
    public TestAutoMode(AutoController ctrl, String name) throws BadParameterTypeException, MissingParameterException {
        super(ctrl, name) ;

        ISettingsSupplier parser = ctrl.getRobot().getSettingsSupplier() ;
        which_ = parser.get(Which).getInteger() ;

        parameters_ = new HashMap<>() ;

        for(String key : parser.getAllKeys(TestModeKey)) {

            if (key.equals("enabled") || key.equals("which"))
                continue ;
                
            int testno ;

            try {
                List<String> children = parser.getAllKeys(TestModeKey + ":" + key) ;
                testno = Integer.parseInt(key) ;

                Map<String, SettingsValue> values  = new HashMap<String, SettingsValue>() ;
                for(String child : children) {
                    SettingsValue v = parser.get(TestModeKey + ":" + key + ":" + child) ;
                    values.put(child, v) ;
                }

                parameters_.put(testno, values) ;
            }
            catch(Exception ex) {
                continue ;                
            }
        }
    }

    /// \brief Returns the number of the test to run
    /// \returns the number of the test to run.
    protected int getTestNumber() {
        return which_;
    }

    /// \brief Returns the double value of a given parameter from the settings file test mode settings
    /// \returns the name value from the settings file    
    protected double getDouble(String name) throws Exception {
        if (!parameters_.containsKey(which_)) {
            throw new Exception("settings file does not contain test parameters for test number " + which_) ;
        }

        Map<String, SettingsValue> params = parameters_.get(which_) ;
        if (!params.containsKey(name)) {
            throw new Exception("settings file contains parameters for test " + which_ + ", but not the parameter '" + name + "'") ;
        }

        double ret = 0.0 ;
        SettingsValue v = params.get(name) ;
        if (v.isInteger()) {
            ret = v.getInteger() ;
        }
        else if (v.isDouble()) {
            ret = v.getDouble() ;
        }
        else if (v.isBoolean()) {
            throw new Exception("the parameter '" + name + "' for test " + which_ + " is a boolean and not a double") ;
        }
        else if (v.isString()) {
            throw new Exception("the parameter '" + name + "' for test " + which_ + " is a string and not a double") ;            
        }

        return ret ;
    }

    /// \brief Returns the double value of a given parameter from the settings file test mode settings
    /// \returns the name value from the settings file    
    protected int getInteger(String name) throws Exception {
        if (!parameters_.containsKey(which_)) {
            throw new Exception("settings file does not contain test parameters for test number " + which_) ;
        }

        Map<String, SettingsValue> params = parameters_.get(which_) ;
        if (!params.containsKey(name)) {
            throw new Exception("settings file contains parameters for test " + which_ + ", but not the parameter '" + name + "'") ;
        }

        int ret = -1 ;
        SettingsValue v = params.get(name) ;
        if (v.isInteger()) {
            ret = v.getInteger() ;
        }
        else if (v.isDouble()) {
            throw new Exception("the parameter '" + name + "' for test " + which_ + " is a double and not a double") ;
        }
        else if (v.isBoolean()) {
            throw new Exception("the parameter '" + name + "' for test " + which_ + " is a boolean and not a double") ;
        }
        else if (v.isString()) {
            throw new Exception("the parameter '" + name + "' for test " + which_ + " is a string and not a double") ;            
        }

        return ret ;
    }

    protected String getString(String name) throws Exception {
        if (!parameters_.containsKey(which_)) {
            throw new Exception("settings file does not contain test parameters for test number " + which_) ;
        }

        Map<String, SettingsValue> params = parameters_.get(which_) ;
        if (!params.containsKey(name)) {
            throw new Exception("settings file contains parameters for test " + which_ + ", but not the parameter '" + name + "'") ;
        }

        String ret = null ;
        SettingsValue v = params.get(name) ;
        if (v.isInteger()) {
            throw new Exception("the parameter '" + name + "' for test " + which_ + " is a integer and not a string") ;
        }
        else if (v.isDouble()) {
            throw new Exception("the parameter '" + name + "' for test " + which_ + " is a double and not a string") ;
        }
        else if (v.isBoolean()) {
            throw new Exception("the parameter '" + name + "' for test " + which_ + " is a boolean and not a string") ;
        }
        else if (v.isString()) {
            ret = v.getString() ;         
        }

        return ret ;
    }
}