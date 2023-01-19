package org.xero1425.base.subsystems.oi ;

/// \file

/// \brief This class represents a button on the OI. This is a logical button which is tied to
/// a physical button.  There is a mapping from the physical button to the logical button item which is
/// given by the ButtonType.
public class OIPanelButton extends OIPanelItem
{
    // The value of the button item
    int value_ ;

    // The previous value of the physical button
    boolean prev_ ;

    // The type of button item
    ButtonType type_ ;

    /// \brief the type of button
    public enum ButtonType {
        Level,              ///< The value of this button item always follows the value of the physical button
        LevelInv,           ///< The value of this button item is the opposite of the value of the physical button
        LowToHigh,          ///< The value of this buttom item is "1" for a single robot loop after the physical button transitions from low to high
        HighToLow           ///< The value of this buttom item is "1" for a single robot loop after the physical button transitions from high to low
    } ;

    /// \brief Create a new button item
    /// \param item the item number for this item
    /// \param type the type of button item
    public OIPanelButton(int item, ButtonType type) {
        super(item, OIPanelItem.JoystickResourceType.Button) ;
        type_ = type ;
        value_ = 0 ;
        prev_ = false ;
    }

    /// \brief returns the type of button item
    /// \returns the type of button item
    public ButtonType getType() {
        return type_ ;
    }

    /// \brief sets the value of the button item given the value of the physical button
    /// \param value the value of the physical button
    @Override
    public void setButtonValue(boolean value) {
        switch(type_)
        {
            case Level:
                value_ = value ? 1 : 0 ;
                break ;

            case LevelInv:
                value_ = value ? 0 : 1 ;
                break ;

            case LowToHigh:
                if (!prev_ && value)
                    value_ = 1 ;
                else
                    value_ = 0 ;
                break ;
                
            case HighToLow:
                if (prev_ && !value)
                    value_ = 1 ;
                else
                    value_ = 0 ;
                break ;
        }

        prev_ = value ;
    }

    /// \brief returns the value of the logical button item
    /// \returns the value of the logical button item
    @Override
    public int getValue() {
        return value_ ;
    }
}