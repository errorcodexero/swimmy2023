package org.xero1425.misc ;

/// \file

/// \brief This class represents a value of type int, long, double, boolean, or string.
public class SettingsValue
{
    //
    // The type of value stored
    //
    private SettingsType type_ ;

    //
    // the value if its an integer
    //
    private int ivalue_ ;

    //
    // The value if its a double
    //
    private double dvalue_ ;

    //
    // The value if its a boolean
    //
    private boolean bvalue_ ;

    //
    // THe value if its a string
    //
    private String svalue_ ;

    /// \brief the type of the value
    public enum SettingsType
    {
        Integer,                ///< an integer value
        Double,                 ///< a double value
        Boolean,                ///< a boolean value
        String                  ///< a string value
    } ;

    /// \brief create a new value from an integer
    public SettingsValue(final int v) {
        ivalue_ = v;
        type_ = SettingsType.Integer;
    }

    /// \brief create a new value from a long
    public SettingsValue(final long v) {
        ivalue_ = (int)v ;
        type_ = SettingsType.Integer ;
    }

    /// \brief create a new value from a double    
    public SettingsValue(final double v) {
        dvalue_ = v;
        type_ = SettingsType.Double;
    }

    /// \brief create a new value from a boolean    
    public SettingsValue(final boolean v) {
        bvalue_ = v;
        type_ = SettingsType.Boolean;
    }

    /// \brief create a new value from a string    
    public SettingsValue(final String v) {
        svalue_ = v ;
        type_ = SettingsType.String ;
    }

    /// \brief return the type of this value
    /// \returns the type of this value
    public SettingsType getType() {
        return type_ ;
    }

    /// \brief returns true if the type is integer
    /// \returns true if the type is integer
    public boolean isInteger() {
        return type_ == SettingsType.Integer ;
    }

    /// \brief returns true if the type is double
    /// \returns true if the type is double
    public boolean isDouble() {
        return type_ == SettingsType.Double ;
    }

    /// \brief returns true if the type is boolean
    /// \returns true if the type is boolean    
    public boolean isBoolean() {
        return type_ == SettingsType.Boolean ;
    }

    /// \brief returns true if the type is string
    /// \returns true if the type is string      
    public boolean isString() {
        return type_ == SettingsType.String ;
    }

    /// \brief returns an integer from the value.
    /// \exception throws a BadParameterTypeException if the SettingsValue is not an integer
    /// \returns the integer value
    public int getInteger() throws BadParameterTypeException {
        if (type_ != SettingsType.Integer)
            throw new BadParameterTypeException(SettingsType.Integer, type_) ;

        return ivalue_ ;
    }

    /// \brief returns an double from the value.
    /// \exception throws a BadParameterTypeException if the SettingsValue is not a double
    /// \returns the double value
    public double getDouble() throws BadParameterTypeException {
        if (type_ != SettingsType.Double && type_ != SettingsType.Integer)
            throw new BadParameterTypeException(SettingsType.Double, type_) ;

        if (type_ == SettingsType.Integer)
            return (double)ivalue_ ;
            
        return dvalue_ ;
    }
    
    /// \brief returns an boolean from the value.
    /// \exception throws a BadParameterTypeException if the SettingsValue is not a boolean
    /// \returns the boolean value    
    public boolean getBoolean() throws BadParameterTypeException {
        if (type_ != SettingsType.Boolean)
            throw new BadParameterTypeException(SettingsType.Boolean, type_) ;
        
        return bvalue_ ;
    }
    
    /// \brief returns an string from the value.
    /// \exception throws a BadParameterTypeException if the SettingsValue is not a string
    /// \returns the string value      
    public String getString() throws BadParameterTypeException {
        if (type_ != SettingsType.String)
            throw new BadParameterTypeException(SettingsType.String, type_) ;
        
        return svalue_ ;
    }    

    /// \brief converts the value to a human readable string
    /// \returns a human readable string
    public String toString() {
        String ret = "[" ;
        ret += type_.toString() ;
        ret += " " ;
        switch(type_) {
            case Integer:
                ret += Integer.toString(ivalue_) ;
                break; 

            case Double:
                ret += Double.toString(dvalue_) ;
                break ;

            case Boolean:
                ret += Boolean.toString(bvalue_) ;
                break ;

            case String:
                ret += "'" + svalue_ + "'" ;
                break ;
        }
        ret += "]" ;
        return ret ;
    }

    /// \brief returns true if two SettingsValue are the same
    /// \returns true if two SettingsValue are the same
    @Override
    public boolean equals(Object obj) {
        if (obj == null)
            return false ;

        if (!(obj instanceof SettingsValue))
            return false ;

        SettingsValue other = (SettingsValue)obj ;
        if (other.type_ != type_)
            return false ;

        boolean ret = false ;

        switch(type_) {
            case Integer:
                ret = (ivalue_ == other.ivalue_) ;
                break ;
            case Double:
                ret = (Math.abs(dvalue_ - other.dvalue_) < 1e-6) ;
                break ;
            case String:
                ret = svalue_.equals(other.svalue_) ;
                break ;
            case Boolean:
                ret = (bvalue_ == other.bvalue_) ;
                break ;
        }

        return ret ;
    }

} ;