package org.xero1425.base.misc;

import com.revrobotics.ColorSensorV3;

import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.Subsystem;
import org.xero1425.misc.BadParameterTypeException;
import org.xero1425.misc.MissingParameterException;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensorSubsystem extends Subsystem {
    static public final String ColorSensorMuxSimDevName = "sensor-mux" ;
    static public final String ColorSensorSimRedValueName = "sensor-mux-red" ;
    static public final String ColorSensorSimGreenValueName = "sensor-mux-green" ;
    static public final String ColorSensorSimBlueValueName = "sensor-mux-blue" ;
    static public final String ColorSensorSimProximityValueName = "sensor-mux-proximity" ;
    static public final String ColorSensorSimIRValueName = "sensor-mux-ir" ;

    private I2C.Port port_ ;
    private int which_ ;
    private I2C muxdev_ ;
    private int muxaddr_ ;
    private byte [] data_ = new byte[1] ;
    private int count_ ;
    private ColorSensorV3 [] sensors_ ;
    private int[] proximity_ ;
    private int[] ir_ ;
    private Color [] colors_ ;
    private boolean running_ ;
    private boolean read_proximity_ ;
    private boolean read_ir_ ;

    private SimDevice i2c_mux_ ;
    private SimDouble[] i2c_mux_red_value_ ;
    private SimDouble[] i2c_mux_green_value_ ;
    private SimDouble[] i2c_mux_blue_value_ ;
    private SimInt[] i2c_mux_proximity_value_ ;
    private SimInt[] i2c_mux_ir_value_ ;

    public ColorSensorSubsystem(Subsystem parent, String name, I2C.Port port) throws BadParameterTypeException, MissingParameterException {
        super(parent, name) ;

        port_ = port ;
        which_ = -1 ;    
        running_ = false ;
        read_proximity_ = false ;
        read_ir_ = false ;

        count_ = getSettingsValue("hw:i2cmux:count").getInteger() ;
        muxaddr_ = getSettingsValue("hw:i2cmux:address").getInteger() ;
       
        if (XeroRobot.isSimulation()) {
            i2c_mux_ = SimDevice.create(ColorSensorMuxSimDevName) ;

            i2c_mux_red_value_ = new SimDouble[count_] ;
            i2c_mux_green_value_ = new SimDouble[count_] ;
            i2c_mux_blue_value_ = new SimDouble[count_] ;
            i2c_mux_proximity_value_ = new SimInt[count_] ;
            i2c_mux_ir_value_ = new SimInt[count_] ;

            for(int i = 0 ;i < count_ ; i++) {
                i2c_mux_red_value_[i] = i2c_mux_.createDouble(ColorSensorSimRedValueName + i, SimDevice.Direction.kBidir, 0) ;
                i2c_mux_green_value_[i] = i2c_mux_.createDouble(ColorSensorSimGreenValueName + i, SimDevice.Direction.kBidir, 0) ;
                i2c_mux_blue_value_[i] = i2c_mux_.createDouble(ColorSensorSimBlueValueName + i, SimDevice.Direction.kBidir, 0) ;
                i2c_mux_proximity_value_[i] = i2c_mux_.createInt(ColorSensorSimProximityValueName + i, SimDevice.Direction.kBidir, 0) ;
                i2c_mux_ir_value_[i] = i2c_mux_.createInt(ColorSensorSimIRValueName + i, SimDevice.Direction.kBidir, 0) ;
            }
        }

        muxdev_ = new I2C(port_, muxaddr_)  ;
        sensors_ = new ColorSensorV3[count_] ;
        colors_ = new Color[count_] ;
        proximity_ = new int[count_] ;
        ir_ = new int[count_] ;

        if (!XeroRobot.isSimulation()) {
            for(int i = 0 ; i < count_ ; i++)
                init(i) ;
        }
    }

    public int count() {
        return count_ ;
    }

    public Color getColor(int which) {
        return colors_[which] ;
    }

    public int getProximity(int which) {
        return proximity_[which] ;
    }

    public int getIR(int which) {
        return ir_[which] ;
    }

    public void enableIR(boolean b) {
        read_ir_ = b ;
    }

    public void enableProximity(boolean b) {
        read_proximity_ = b ;
    }

    @Override
    public void computeMyState() {
        running_ = true ;

        for(int i = 0 ; i < count_ ; i++) {
            select(i) ;

            colors_[i] = getColor() ;

            if (read_proximity_) {
                proximity_[i] = getProximity();
            }

            if (read_ir_) {
                ir_[i] = getIR() ;
            }
        }
    }

    // Derived class override this for different initialization
    protected ColorSensorV3.ColorSensorResolution getResolution(int which) {
        return ColorSensorV3.ColorSensorResolution.kColorSensorRes13bit;
    }

    // Derived class override this for different initialization    
    protected ColorSensorV3.ColorSensorMeasurementRate getMeasurementRate(int which) { 
        return ColorSensorV3.ColorSensorMeasurementRate.kColorRate25ms ;
    }

    // Derived class override this for different initialization    
    protected ColorSensorV3.GainFactor getGain(int which) {
        return ColorSensorV3.GainFactor.kGain18x ;
    }

    private Color getColor() {
        Color c = Color.kBlack ;

        if (XeroRobot.isSimulation()) {
            if (running_) {
                double r = i2c_mux_red_value_[which_].get() ;
                double g = i2c_mux_green_value_[which_].get() ;
                double b = i2c_mux_blue_value_[which_].get() ;
                c = new Color(r, g, b) ;
            }
        }
        else {
            c = sensors_[which_].getColor() ;
        }

        return c ;
    }

    private int getProximity() {
        int p = 0 ;

        if (XeroRobot.isSimulation()) {
            if (running_) {
                p = i2c_mux_proximity_value_[which_].get() ;
            }
        }
        else {
            p = sensors_[which_].getProximity() ;
        }

        return p ;
    }

    private int getIR() {
        int p = 0 ;

        if (XeroRobot.isSimulation()) {
            if (running_) {
                p = i2c_mux_ir_value_[which_].get() ;
            }
        }
        else {
            p = sensors_[which_].getIR() ;
        }

        return p ;
    }

    private void init(int which) {
        select(which) ;
        
        sensors_[which] = new ColorSensorV3(port_) ;

        ColorSensorV3.ColorSensorResolution res = getResolution(which) ;
        ColorSensorV3.ColorSensorMeasurementRate rate = getMeasurementRate(which) ;
        ColorSensorV3.GainFactor gain = getGain(which) ;
        sensors_[which].configureColorSensor(res, rate, gain) ;        
    }

    private void select(int which) {
        if (muxdev_ != null && which != which_) {
            data_[0] = (byte)(0x01 << which) ;
            muxdev_.writeBulk(data_);
            which_ = which ;
        }
    }
}
