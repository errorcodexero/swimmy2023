package frc.robot.automodes;

import org.xero1425.base.controllers.AutoController;
import org.xero1425.base.controllers.TestAutoMode;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

public class SwimmyTestAutoMode extends TestAutoMode {

    public SwimmyTestAutoMode(AutoController ctrl)
            throws BadParameterTypeException, MissingParameterException {
        super(ctrl, "Swimmy-Test-Mode");

        switch (getTestNumber()) {
            case 0: 
                
        }
    }
    
}
