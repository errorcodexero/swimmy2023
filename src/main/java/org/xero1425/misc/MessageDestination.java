package org.xero1425.misc ;

/// \file

/// \brief an interface that defines the required functio of a message definition
public interface MessageDestination
{
    /// \brief display a message
    /// \param type the message type
    /// \param subsystem the subsystem ID for the message
    /// \param msg the text of the messag
    public abstract void displayMessage(MessageType type, int subsystem, String msg) ;
}
