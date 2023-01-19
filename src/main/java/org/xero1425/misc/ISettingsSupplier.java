package org.xero1425.misc;

import java.util.List;

/// \file

/// \brief This interface defines requirements for a class that provide settings to the robot
public interface ISettingsSupplier {
    /// \brief Returns true if a setting with the given name is present, otherwise is returns false
    /// \param name the name of the setting to check
    /// \returns true if a setting with the given name is present, otherwise is returns false
    boolean isDefined(String name)  ;

    /// \brief Return a SettingsValue given the settings name.
    /// \exception throws MissingParameterException if the name does not map to an entry in the JSON file
    /// \param name the name of the setting to retreive.
    /// \returns the SettingsValue object that for the setting with the given name    
    SettingsValue get(String name) throws MissingParameterException ;

    /// \brief Return a SettingsValue given the settings name, or returns null if it does not exist
    /// \param name the name of the setting to retreive.
    /// \returns the SettingsValue object that for the setting with the given name, or null if it does not exist    
    SettingsValue getOrNull(String name) ;

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
    List<String> getAllKeys(String path) ;
}
