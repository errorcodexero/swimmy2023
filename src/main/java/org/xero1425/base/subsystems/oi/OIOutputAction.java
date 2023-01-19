package org.xero1425.base.subsystems.oi;

import org.xero1425.base.actions.Action;

public class OIOutputAction extends Action {
    private OISubsystem sub_ ;
    private int device_ ;
    private int output_ ;
    private boolean value_ ;

    public OIOutputAction(OISubsystem sub, int device, int output, boolean value) {
        super(sub.getRobot().getMessageLogger());

        sub_ = sub ;
        device_ = device ;
        output_ = output ;
        value_ = value ;
    }

    public int getDevice() {
        return device_ ;
    }

    public int getOutput() {
        return output_ ;
    }

    public boolean getValue() {
        return value_ ;
    }

    @Override
    public void start() {
        OIDevice dev = sub_.getDevice(device_) ;
        dev.setOutput(output_, value_) ;
    }

    @Override
    public void run() {

    }

    @Override
    public String toString(int indent) {
        return spaces(indent) + "OIOuptutAction(" + device_ + ", " + output_ + ", " +
            (value_ ? "true" : "false") ;
    }
}
