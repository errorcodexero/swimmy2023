package org.xero1425.base.actions ;

/// \file

/// \brief this class is an exception that is thrown when an invalid request is made of an action.
public class InvalidActionRequest extends Exception
{
    // The action that was invalid
    private Action act_ ;

    // The reason it was invalid
    private Reason reason_ ;

    static final long serialVersionUID = 42 ;

    /// \brief The reason why an action was invalid
    public enum Reason
    {
        ModifyingRunningAction          ///< Modifying an action while it is running where that is not allowed
    } ;
    
    /// \brief create the InvalidActionRequest object.
    /// \param act the action that caused the exception
    /// \param reason the reason the action request was invalid
    /// \param msg a string describing the error that occurred
    public InvalidActionRequest(Action act, Reason reason, String msg) {
        super(act.toString() + "- " + msg) ;

        act_ = act ;
        reason_ = reason ;
    }

    /// \brief return the action that caused the exception
    /// \returns the action that caused the exception
    public Action getAction() {
        return act_ ;
    }

    /// \brief return the reason for the exception
    /// \returns the reasons for the exception
    public Reason getReason() {
        return reason_ ;
    }
}