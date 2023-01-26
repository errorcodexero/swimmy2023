package org.xero1425.swervelib;

public class SDSModuleGlobalConfig {
    static String canbus_ = "" ;

    static public void setCanBus(String bus) {
        canbus_ = bus ;
    }

    static public String getCanBus() {
        return canbus_ ;
    }
}
