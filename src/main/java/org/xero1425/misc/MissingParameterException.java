package org.xero1425.misc ;

import java.lang.Exception ;

/// \file

/// \brief this exception is thrown when the SettingsParser is asked for a parameter that does not exist
public class MissingParameterException extends Exception
{    
    private String param_ ;

    static final long serialVersionUID = 42 ;
    
    /// \brief create the exception object
    /// \param param the name of the missing parmater
    public MissingParameterException(String param) {
        super("missing parameter '" + param + "'") ;
        param_ = param ;
    }

    /// \brief returns the name of the missing parameter
    /// \returns the name of the missing parameter
    public String getParameter() {
        return param_ ;
    }
}

