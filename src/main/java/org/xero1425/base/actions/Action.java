package org.xero1425.base.actions ;

import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType; 

/// \file

/// \brief This is the base classs for an action that is executed by a Subsystem.
public abstract class Action
{
    /// \brief the name to use with the message logger for actions
    public static final String LoggerName = "actions" ;
    
    // If true, the action is complete
    private boolean done_ ;

    // if true, the action was canceled
    private boolean canceled_ ;

    // A reference to the message logger
    private MessageLogger logger_ ;
    
    // The unique ID for the action
    private int id_ ;

    // The ID for the next action created
    private static int current_id_ = 0 ;

    // The logger ID for the action class
    private static int logger_id_ = -1 ;

    /// \brief create a new action
    /// \param logger the message logger for printing messages about the action lifecycle
    public Action(MessageLogger logger) {
        logger_ = logger ;
        id_ = current_id_++ ;
        logger_id_ = getLoggerID(logger) ;

        done_ = false ;
        canceled_ = false ;
    }

    /// \brief start an action
    /// This method is typically overridden by any derived class.  The derived version
    /// this method should call the base class method.
    public void start() throws Exception {
        logger_.startMessage(MessageType.Debug, logger_id_) ;
        logger_.add("starting action: ") ;
        addActionToMessage() ;
        logger_.endMessage();

        done_ = false ;
        canceled_ = false ;
    }

    /// \brief run an action.
    /// This method should be overridden by any derived class.  The derived version of
    /// this method should call the base class method.
    public void run() throws Exception {
    }

    /// \brief return the status of thie action.
    /// \returns true if the action is complete, otherwise false.
    public boolean isDone() {
        return done_ ;
    }

    /// \brief returns true if the action was canceled
    /// \returns true if the action was canceled
    public boolean isCanceled() {
        return canceled_ ;
    }

    /// \brief return the ID for the action.
    /// Every action created is assigned a unique integer ID.  This method returns the integer ID
    /// for this action.
    /// \returns unique integer ID for the action
    public int getID() {
        return id_ ;
    }

    /// \brief return a human readable string describing the action
    /// \returns a human readable string describing the action.
    public abstract String toString(int indent) ;

    /// \brief return the message logger object
    /// \returns the message logger object
    public MessageLogger getMessageLogger() {
        return logger_ ;
    }

    /// \brief cancel the current action.
    /// This is typically overridden by the derived class.  The derived version of
    /// this method should call the base class method.
    public void cancel() {
        if (!isDone()) {
            logger_.startMessage(MessageType.Debug, logger_id_) ;
            logger_.add("canceling action: ") ;
            addActionToMessage() ;
            logger_.endMessage();
            done_ = true ;
            canceled_ = true ;
        }
    }

    /// \brief get the message logger ID for Actions.
    /// This method register for a logger ID from the message logger for the name
    /// "action".  This value is saved and cached and returns in subsequent calls to
    /// this method.
    /// \returns the message logger ID for actions.
    public static int getLoggerID(MessageLogger logger) {
        if (logger_id_ == -1)
            logger_id_ = logger.registerSubsystem(LoggerName) ;

        return logger_id_ ;
    }

    /// \brief called to set the action as complete.  
    /// Called from a derived class when the action is complete.  This sets the
    /// action done state to true.
    protected void setDone() {
        logger_.startMessage(MessageType.Debug, logger_id_) ;
        logger_.add("completing action: ") ;
        addActionToMessage() ;   
        logger_.endMessage();
        done_ = true ;
    }

    /// \brief used to create a string with a set of spaces.
    /// \param n the number of spaces to print
    /// \returns a string with the given number of spaces
    protected String spaces(int n) {
        StringBuilder str = new StringBuilder() ;

        for(int i = 0 ; i < n ; i++)
            str.append(' ') ;

        return str.toString() ;
    }

    /// \brief generate a prefix for an action.
    /// This is generally called from the derived class toString()
    /// method to provide a uniform view of actions.
    protected String prefix(int n) {
        return spaces(n) + id_ + ": " ;
    }

    private void addActionToMessage() {
        String msg = toString(0) ;
        if (msg.indexOf('\n') != -1) {
            logger_.add("\n") ;
        }

        logger_.add(toString(0)) ;
    }    
}
