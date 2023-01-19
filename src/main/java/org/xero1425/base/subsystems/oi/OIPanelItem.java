package org.xero1425.base.subsystems.oi ;

/// \file

/// \brief A base class for various items that can appear on the OI.
public abstract class OIPanelItem {
    
    // The resource type (button or axis)
    private JoystickResourceType type_ ;

    // The item number
    private int item_number_ ;

    /// \brief Type of physical resource that backs the item
    public enum JoystickResourceType
    {
        Axis,           ///< An axis with values from -1.0 to 1.0
        Button          ///< A button with values of true or false
    } ;

    /// \brief create a new item
    /// \param itemno the item number for the item
    /// \param type the type of physical resource for the item
    public OIPanelItem(int itemno, JoystickResourceType type) {
        item_number_ = itemno ;
        type_ = type ;
    }

    /// \brief return the item number for this item
    /// \returns the item number for this item
    public int getItemNumber() {
        return item_number_ ;
    }

    /// \brief return the physical resource type for this item
    /// \returns the physical resource type for this item
    public JoystickResourceType getResourceType() {
        return type_ ;
    }

    /// \brief set the button value for this item
    /// \param value the value to set the button
    public void setButtonValue(boolean value) throws Exception {
        throw new Exception("must override this for button based items") ;
    }

    /// \brief set the axis value for this item
    /// \param value the value for the axis (between -1.0 and 1.0)
    public void setAxisValue(double value) throws Exception {
        throw new Exception("must override this for axis based items") ;
    }

    /// \brief return the value associated with the item
    /// returns the value associated with the item
    public abstract int getValue() ;

} ;