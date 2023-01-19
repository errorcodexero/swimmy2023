package org.xero1425.misc;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.json.simple.JSONObject;


/// \file

/// \brief This class implmements the ISettingsSupplier interface by reading a JSON settings file.
///
/// Settings are stored in a JSON file and are accessed by a single string representing the desired hierarchy.
/// For instance, if the settings file contained the following ...
///
///     subsystems : {
///       tankdrive : {
///          width : 30.0,
///          length : 32.0
///       }        
///     }
///
/// Then the settings name <em>subsystems:tankdrive:width</em> would reference the value 3.0.  Values can be conditional
/// based on defines. See the method addDefine() for more information about this capability.
///
public class JsonSettingsParser implements ISettingsSupplier {
    private MessageLogger logger_ ;
    private List<String> defines_ ;
    private JSONObject contents_ ;

    /// \brief Creates a new JsonSettings parser
    /// \param logger message logger for logging messages while reading the JSON settings file
    public JsonSettingsParser(MessageLogger logger) {
        logger_ = logger ;
        defines_ = new ArrayList<String>() ;
    }

    public boolean readFile(String filename) {
        
        logger_.startMessage(MessageType.Info);
        logger_.add("reading JSON robots setting file ").addQuoted(filename) ;
        logger_.endMessage();    

        contents_ = JsonReader.readFile(filename, logger_) ;
        return contents_ != null ;
    }

    /// \brief add a define to the reading process
    ///
    /// A define is a value that is stored and used in the settings lookup process
    /// to find settings specific to that define.  This is most commonly used for settings that are
    /// specific to the competition robot versus the practice robot.  Either the COMPETITION define is
    /// set or the PRACTICE setting is set.  The value in the JSON file can be of the form
    ///
    ///     subsystems : {
    ///       subname : {
    ///          value : { 
    ///             "PRACTICE" : 1.0,
    ///             "COMPETITION" : 2.0,
    ///          }
    ///       }        
    ///     }
    /// 
    /// With this form, a lookup of the settings value subsystem:subname:value, will pick either PRACTICE or
    /// COMPETITION depending on the define that is set.
    ///
    /// \param name the name of the define to add
    public void addDefine(String name) {
        if (!defines_.contains(name))
            defines_.add(name) ;
    }


    /// \brief Return a SettingsValue given the settings name.
    /// \exception throws MissingParameterException if the name does not map to an entry in the JSON file
    /// \param name the name of the setting to retreive.
    /// \returns the SettingsValue object that for the setting with the given name
    @Override
    public SettingsValue get(String name) throws MissingParameterException {
        SettingsValue v = getOrNull(name) ;

        if (v == null)
            throw new MissingParameterException(name) ;

        return v ;
    }

    /// \brief Returns true if a setting with the given name is present, otherwise is returns false
    /// \param name the name of the setting to check
    /// \returns true if a setting with the given name is present, otherwise is returns false
    @Override
    public boolean isDefined(String name) {
        SettingsValue v = getOrNull(name) ;
        return v != null ;
    }

    /// \brief Return a SettingsValue given the settings name, or returns null if it does not exist
    /// \param name the name of the setting to retreive.
    /// \returns the SettingsValue object that for the setting with the given name, or null if it does not exist
    @Override
    public SettingsValue getOrNull(String name) {
        String [] parts = name.split(":") ;
        SettingsValue v = null ;

        JSONObject current = findParent(parts) ;
        if (current == null)
            return null ;

        Object value = current.get(parts[parts.length-1]) ;
        if (value == null)
            return null ;

        if ((value instanceof JSONObject)) {
            //
            // This might be a conditional definiton based on a define
            //
            JSONObject condobj = (JSONObject)value ;
            for(String define : defines_)
            {
                if (condobj.containsKey(define))
                {
                    value = condobj.get(define) ;
                    break ;
                }
            }                
        }

        if ((value instanceof Double) == true) {
            v = new SettingsValue((Double)value) ;
        }
        else if ((value instanceof Integer) == true) {
            v = new SettingsValue((Integer)value) ;
        }
        else if ((value instanceof Long) == true) {
            v = new SettingsValue((Long)value) ;
        }
        else if ((value instanceof Boolean) == true) {
            v = new SettingsValue((Boolean)value) ;
        }
        else if ((value instanceof String) == true) {
            v = new SettingsValue((String)value) ;
        }

        return v;
    }

    /// \brief For a given setting entry in the JSON file, return all children
    ///
    /// This method returns a list of key given a parent key.
    ///
    ///     subsystems : {
    ///       tankdrive : {
    ///          width : 30.0,
    ///          length : 32.0,
    ///          inches_per_tick: 0.006
    ///       }        
    ///     } 
    ///
    /// For instance with the setup above, getAllKeys("subsystems:tankdrive") will return
    /// a list that contains "width", "length", and "inches_per_tick"
    ///    
    /// \param path the name of the key to query for children
    /// \returns a list of keys that are chidren of the key given
    @Override
    public List<String> getAllKeys(String path) {
        String [] parts = path.split(":") ;
        JSONObject parent = findParent(parts) ;

        List<String> ret = new ArrayList<String>() ;

        if (parent == null)
            return null ;

        JSONObject obj = (JSONObject)parent.get(parts[parts.length - 1]) ;
        if (obj != null) {
            @SuppressWarnings("unchecked")
            Set<String> keys = (Set<String>)obj.keySet() ;
            if (keys != null) {
                for(String key : keys)
                    ret.add(key) ;
            }
        }

        return ret ;
    }

    private JSONObject findParent(String [] parts) {
        JSONObject current = contents_ ;
        int index = 0 ;

        while (index < parts.length - 1)
        {
            Object obj = current.get(parts[index]) ;
            if (obj == null)
                return null ;

            if (!(obj instanceof JSONObject))
                return null ;

            current = (JSONObject)obj ;
            index++ ;
        }

        return current ;
    }

}
