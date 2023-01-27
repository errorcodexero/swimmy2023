package org.xero1425.base.subsystems.oi;

import java.util.ArrayList;
import java.util.List;

import org.xero1425.base.controllers.AutoMode;
import org.xero1425.base.controllers.TestAutoMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class OIShuffleBoardDevice extends OIDevice {
    private SendableChooser<Integer> automode_chooser_ ;
    private SendableChooser<Integer> testmode_chooser_ ;

    private class ModeNameNumber
    {
        public final String Name ;
        public final int Number ;

        public ModeNameNumber(String name, int num) {
            Name = name ;
            Number = num ;
        }
    }

    public OIShuffleBoardDevice(OISubsystem sub) {
        super(sub, "shuffleboard-device") ;
        automode_chooser_ = null ;
        testmode_chooser_ = null ;
    }

    @Override
    public int getAutoModeSelector() {
        
        int sel = Integer.MAX_VALUE ;

        if (automode_chooser_ != null) {
            Integer obj = automode_chooser_.getSelected() ;
            if (obj != null) {
                sel = obj ;
            }
        }

        if (testmode_chooser_ != null && sel == 0 && !DriverStation.isFMSAttached()) {
            Integer obj = testmode_chooser_.getSelected() ;
            if (obj != null) {
                sel = obj ;
            }
        }

        return sel ;
    }

    @Override
    public void computeState() {
    }

    @Override
    public void initAutoModes() {

        List<ModeNameNumber> testmodes = getTestModes() ;
        List<ModeNameNumber> automodes = getAutoModes() ;

        automode_chooser_ = new SendableChooser<Integer>() ;
        for(int i = 0 ; i < automodes.size() ; i++) {
            ModeNameNumber n = automodes.get(i) ;
            Integer iobj = Integer.valueOf(n.Number) ;
            automode_chooser_.addOption(Integer.toString(n.Number) + " - " + n.Name, iobj) ;
            if (i == 0) {
                automode_chooser_.setDefaultOption(Integer.toString(n.Number) + " - " + n.Name, iobj);
            }
        }
        Shuffleboard.getTab("AutoMode").add("AutoMode", automode_chooser_).withSize(2,1).withWidget(BuiltInWidgets.kComboBoxChooser) ;

        if (!DriverStation.isFMSAttached()) {
            testmode_chooser_ = new SendableChooser<Integer>() ;
            for(int i = 0 ; i < testmodes.size() ; i++) {
                ModeNameNumber n = testmodes.get(i) ;
                Integer iobj = Integer.valueOf(n.Number) ;
                testmode_chooser_.addOption(Integer.toString(-n.Number) + " - " + n.Name, iobj) ;
                if (i == 0) {
                    testmode_chooser_.setDefaultOption(Integer.toString(-n.Number) + " - " + n.Name, iobj);
                }
            }
            Shuffleboard.getTab("AutoMode").add("TestMode", testmode_chooser_).withSize(2,1).withWidget(BuiltInWidgets.kComboBoxChooser) ;
        }
    }

    private List<ModeNameNumber> getAutoModes() {
        List<AutoMode> modes = getSubsystem().getRobot().getAllAutomodes() ;
        List<ModeNameNumber> modenames = new ArrayList<ModeNameNumber>() ;

        String numzero = "Test Mode" ;

        if (DriverStation.isFMSAttached()) {
            numzero = "None" ;
        }

        modenames.add(new ModeNameNumber(numzero, 0)) ;
        for(int i = 0 ; i < modes.size() ; i++) {
            ModeNameNumber n = new ModeNameNumber(modes.get(i).getName(), i + 1) ;
            modenames.add(n) ;
        }

        return modenames ;
    }

    private List<ModeNameNumber> getTestModes() {
        List<ModeNameNumber> modes = new ArrayList<ModeNameNumber>() ;

        if (!DriverStation.isFMSAttached()) {
            //
            // We don't provide test modes if we are connected to the field
            //
            TestAutoMode mode = getSubsystem().getRobot().getTestAutoMode() ;
            for(int i = 1 ; i < 200 ; i++) {
                try {
                    String result = mode.setTestModeTestNumber(i) ;
                    if (result.length() > 0) {
                        ModeNameNumber m = new ModeNameNumber(result, -i) ;
                        modes.add(m) ;
                    }
                }
                catch(Exception ex) {
                }
            }
        }

        return modes ;
    }
}
