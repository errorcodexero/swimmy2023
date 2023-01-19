
package org.xero1425.misc ;

/// \file

/// \brief this interface defines the requirements of a time source for the MessageLogger
public interface MessageTimeSource 
{
    /// \brief return the time in seconds since the start of the robot code
    /// \returns the time in seconds since the start of robot code
    public double getTime() ;
} ;
