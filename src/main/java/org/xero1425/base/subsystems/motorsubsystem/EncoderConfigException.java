package org.xero1425.base.subsystems.motorsubsystem ;

/// \file

/// \brief This class is an exception that is thrown when there is an encoder
/// configuration error.
public class EncoderConfigException extends Exception
{
    static final long serialVersionUID = 42 ;
 
    /// \brief Create the exception
    /// \param msg string describing the configuration error
    public EncoderConfigException(String msg) {
        super(msg) ;
    }
}
