package org.xero1425.misc ;

import java.lang.Exception ;

/// \file 

/// \brief This exception is thrown when a parameter value the wrong type
public class BadParameterTypeException extends Exception
{
    //
    // The expected type
    //    
    private SettingsValue.SettingsType expected_ ;

    //
    // The actual type
    //
    private SettingsValue.SettingsType got_ ;

    static final long serialVersionUID = 42 ;
    
    /// \brief create the excpetion object to throw
    /// \param expected the expected type
    /// \param got the actual type
    public BadParameterTypeException(SettingsValue.SettingsType expected, SettingsValue.SettingsType got) {
        super("wrong parameter type, expected " + expected.toString() + " got " + got.toString()) ;
        expected_ = expected ;
        got_ = got ;
    }

    /// \brief returns the expected type
    /// \returns the expected type
    public SettingsValue.SettingsType expected() {
        return expected_ ;
    }

    /// \brief returns the actual type
    /// \returns the actual type
    public SettingsValue.SettingsType got() {
        return got_ ;
    }

}

