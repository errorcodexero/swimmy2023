package org.xero1425.misc ;

/// \file

/// \brief this exception is through when a requested path is not found in the path manager
public class MissingPathException extends Exception
{
    private String name_ ;

    static final long serialVersionUID = 42 ;
    
    /// \brief create the exception to throw
    /// \param name the name of the missing path
    public MissingPathException(String name) {
        super("path '" + name + "' does not exist") ;

        name_ = name ;
    }

    /// \brief returns the name of the missing path
    /// \returns the name of the missing path
    public String getMissingPathName() {
        return name_ ;
    }
}
