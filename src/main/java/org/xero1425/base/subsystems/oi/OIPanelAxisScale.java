package org.xero1425.base.subsystems.oi ;

/// \file

/// \brief an item that maps a joystick axis value to an integer value.
/// The mapping is done by having the caller supply an array of ascending values that are
/// passed to the constructor of the object.
///
///     Double [] map = { -0.9, -0.75, -0.5, -0.25, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 } ;
///
/// The value is determined by comparing the value read from the joystick to the values in 
/// the array.  The first value in the array that is greater than the axis value is the value of the item.
///
public class OIPanelAxisScale extends OIPanelItem
{
    // The value of the item
    private int value_ ;

    // The set of ranges for the item
    final private Double[] map_ ;

    /// \brief Create a new OIPanelAxisScale item
    /// \param item the item number for the item
    /// \param map the map of axis values to item number
    public OIPanelAxisScale(int item, final Double[] map) {
        super(item, OIPanelItem.JoystickResourceType.Axis) ;

        map_ = map ;
        value_ = -1 ;
    }

    /// \brief set the axis value given a raw joystick axis value
    /// \param value the raw axis value
    @Override
    public void setAxisValue(double value) {
        value_ = map_.length;
        for(int i = 0 ; i < map_.length ; i++)
        {
            if (value <= map_[i]) {
                value_ = i ;
                break ;
            }
        }
    }

    /// \brief return the computed value for the item
    /// \returns the computed value for the item
    @Override
    public int getValue() {
        return value_ ;
    }
}
