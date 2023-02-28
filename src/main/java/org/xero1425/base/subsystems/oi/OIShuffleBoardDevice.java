package org.xero1425.base.subsystems.oi;

import java.util.List;

import org.xero1425.base.controllers.AutoMode;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class OIShuffleBoardDevice extends OIDevice {
    private SendableChooser<Integer> chooser_ ;
    private int automode_ ;

    public OIShuffleBoardDevice(OISubsystem sub) {
        super(sub, "shuffleboard-device") ;
        chooser_ = null ;
        automode_ = -1 ;
    }

    @Override
    public int getAutoModeSelector() {
        return automode_ ;
    }

    @Override
    public void disabledProcessing() {
    }

    @Override
    public void computeState() {
        List<AutoMode> modes = getSubsystem().getRobot().getAllAutomodes() ;

        if (chooser_ == null && modes.size() > 0) {
            chooser_ = new SendableChooser<Integer>() ;
            for(int i = 0 ; i < modes.size() ; i++) {
                AutoMode mode = modes.get(i) ;
                chooser_.addOption(Integer.toString(i) + ":" + mode.getName(), Integer.valueOf(i)) ;
                if (i == 0) {
                    chooser_.setDefaultOption(Integer.toString(i) + ":" + mode.getName(), Integer.valueOf(i));
                }
            }

            Shuffleboard.getTab("AutoMode").add("AutoMode", chooser_).withSize(2,1).withWidget(BuiltInWidgets.kComboBoxChooser) ;
        }
        else if (chooser_ != null) {
            int num = chooser_.getSelected() ;
            if (num != automode_) {
                automode_ = num ;
            }
        }
    }
}
