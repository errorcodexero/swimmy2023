package org.xero1425.base.subsystems.oi ;

/// \file

/// \brief This class is a logical axis switch which maps a physical axis with
/// a value from -1.0 to 1.0 to a evenly spaced set of ranges.  The count gives the number
/// of range.  For instance, if the count was set to 4, then the axis values would mape as follows
///
///     -1.0 to -0.5   -> 0
///     -0.5 to 0.0    -> 1
///      0.0 to 0.5    -> 2
///      0.5 to 1.0    -> 3
///     
public class OIPanelAxisSwitch extends OIPanelItem
{   
    // The computed value of the item
    private int value_ ;

    // The number of ranges 
    private int count_ ;

    /// \brief Create a new OIPanelAxisSwitch item
    /// \param item the item number for this item
    /// \param count the number of ranges for this item
    public OIPanelAxisSwitch(int item, int count) {
        super(item, OIPanelItem.JoystickResourceType.Axis) ;

        count_ = count ;
    }

    /// \brief returns the number of ranges for this item
    /// \returns the number of ranges for this item
    public int getCount() {
        return count_ ;
    }

    /// \brief set the value for this item given the value of the axis
    /// \param value the value of the assocaited axis
    @Override
    public void setAxisValue(double value) {
        double minvalue = -1.0 ;
        double maxvalue = 1.0 ;
        double range = maxvalue - minvalue ;
        double slice = range / count_ ;

        double v = minvalue + slice ;
        value_ = count_ - 1 ;
        for(int i = 0 ; i < count_ ; i++) {
            if (value < v) {
                value_ = i ;
                break ;
            }
            v += slice ;
        }
    }

    /// \brief return the value for this item
    /// \returns the value for this item
    @Override
    public int getValue() {
        return value_ ;
    }
}