package org.xero1425.base.subsystems.vision;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;
import org.xero1425.misc.MessageLogger ;
import org.xero1425.misc.MessageType ;
import org.xero1425.base.IVisionAlignmentData;
import org.xero1425.base.IVisionLocalization;
import org.xero1425.base.XeroRobot;
import org.xero1425.base.subsystems.Subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class LimeLightSubsystem extends Subsystem implements IVisionLocalization, IVisionAlignmentData {
    public final static String LimeLightTableName = "limelight";
    public final static String CamModeKeyName = "camMode" ;
    public final static String LedModeKeyName = "ledMode" ;
    public final static String PipelineKeyName = "pipeline" ;

    public final static boolean TestTagZHeight = true ;
    public final static boolean TestHeadingVersusDB = true ;


    public class Retro {
        public Translation2d pts[] ;
        public Pose3d camToTarget ;
        public Pose3d robotToField ;
        public Pose3d robotToTarget ;
        public Pose3d targetToCamera ;
        public Pose3d targetToRobot ;
        public double ta ;
        public double tx ;
        public double txp ;
        public double ty ;
        public double typ ;
    } ;

    public class Fiducial {
        public int id ;
        public String family ;
        public Pose3d camToTarget ;
        public Pose3d robotToField ;
        public Pose3d robotToTarget ;
        public Pose3d targetToCamera ;
        public Pose3d targetToRobot ;
        public double ta ;
        public double tx ;
        public double txp ;
        public double ty ;
        public double typ ;
    } ;

    public class Detector {
        public String className ;
        public int classId ;
        public double confidence ;
        public Translation2d [] pts ;
        public double ta ;
        public double tx ;
        public double txp ;
        public double ty ;
        public double typ ;
    } ;

    public class Classifier {
        public String className ;
        public int classId ;
        public double confidence ;
    }

    private boolean found_ ;
    private int id_ ;
    private double tl_ ;
    private double cl_ ;
    private double ts_ ;
    private boolean valid_targets_ ;

    private Retro [] retro_ ;
    private Fiducial[] fuds_ ;
    private Detector[] detectors_ ;
    private Classifier[] classifiers_ ;

    private Pose3d botpose_ ;
    private Pose3d wpired_ ;
    private Pose3d wpiblue_ ;

    // The network tables entry for the limelight
    private NetworkTable nt_ ;
    private int pipeline_ ;

    private double tx_;
    private double ty_;
    private boolean tv_;

    private CamMode cam_mode_ ;
    private LedMode led_mode_;

    public LimeLightSubsystem(Subsystem parent, String name) {
        super(parent, name) ;

        cam_mode_ = CamMode.Invalid ;
        pipeline_ = -1 ;

        nt_ = NetworkTableInstance.getDefault().getTable(LimeLightTableName) ;

        setPipeline(0);
        setCamMode(CamMode.VisionProcessing);
    }

    ///////////////////////////////////////////////////////
    //
    // The IVisionAlignmentData interface
    //
    ///////////////////////////////////////////////////////

    public double getTX() {
        return tx_ ;
    }

    public double getTY() {
        return ty_ ;
    }

    public boolean isTargetDetected() {
        return tv_ ;
    }

    public void setCamMode(CamMode mode) {
        if (cam_mode_ != mode)
        {
            switch(mode) {
                case VisionProcessing:
                    nt_.getEntry(CamModeKeyName).setNumber(0) ;
                    break ;
                case DriverCamera:
                    nt_.getEntry(CamModeKeyName).setNumber(1) ;
                    break ;
                case Invalid:
                    break ;
            }

            cam_mode_ = mode ;
        }
    }

    public void setLedMode(LedMode mode) {
        if (led_mode_ != mode)
        {
            switch(mode)
            {
                case UseLED:
                    nt_.getEntry(LedModeKeyName).setNumber(0) ;
                    break ;
                case ForceOff:
                    nt_.getEntry(LedModeKeyName).setNumber(1) ;
                    break ;
                case ForceBlink:
                    nt_.getEntry(LedModeKeyName).setNumber(2) ;
                    break ;
                case ForceOn:
                    nt_.getEntry(LedModeKeyName).setNumber(3) ;
                    break ;
                case Invalid:
                    break ;                                                                                
            }

            led_mode_ = mode ;
        }
    }    

    ///////////////////////////////////////////////////////
    //
    // The IVisionAlignmentData interface
    //
    ///////////////////////////////////////////////////////

    public int getTagCount() {
        if (fuds_ == null)
            return 0;

        return fuds_.length;
    }

    private boolean areTagsValid(Pose2d db) {
        if (fuds_ == null || fuds_.length == 0) {
            return false ;
        }

        //
        // Check the Z coordinate of any tag detected and if any of the Z
        // coordinate are on 
        // 
        boolean ret = true ;

        if (TestTagZHeight) {
            for(int i = 0 ; i < fuds_.length ; i++) {
                if (Math.abs(fuds_[i].targetToRobot.getZ()) > 1.0) {
                    ret = false ;
                }
            }
        }

        if (TestHeadingVersusDB) {
            
        }

        return ret ;
    }

    public LocationData getLocation(Pose2d db) {
        LocationData ret = null ;

        if (XeroRobot.isSimulation()) {
            //
            // When debugging, the Double.MAX_VALUE can be replaced with something smaller to make
            // the tag disappear after a fixed amount of time. 
            //
            double t = Timer.getFPGATimestamp();
            if (t > 1.0 && t < Double.MAX_VALUE) {
                found_ = true ;
                valid_targets_ = true;
                fuds_ = new Fiducial[1];
                double ax = Math.toRadians(0.0);
                double ay = Math.toRadians(0.0);
                double az = Math.toRadians(180.0);
                Rotation3d r = new Rotation3d(ax, ay, az);
                wpiblue_ = new Pose3d(3.5, 7.0, 0.333, r);
                tl_ = 0.020;
                cl_ = 0.011;
            }
            else {
                fuds_ = null;
            }
        }

        if (found_ && valid_targets_ && areTagsValid(db)) {
            ret = new LocationData() ;
            ret.location = wpiblue_ ;
            ret.when = getRobot().getTime() - (tl_ + cl_) / 1000.0;
        }

        return ret ;
    }

    public double getDistance() {
        if (XeroRobot.isSimulation()) {
            return 1.5;
        }

        return fuds_[0].robotToTarget.getTranslation().getNorm();
    }

    public double getMultiTagDistance() {
        if (XeroRobot.isSimulation()) {
            return 1.5;
        }

        double dist = Double.MAX_VALUE ;
        for(var fud : fuds_) {
            double zvalue = Math.abs(fud.robotToTarget.getTranslation().getZ()) ;
            if (zvalue < dist) {
                dist = zvalue;
            }
        }

        return dist ;
    }

    ///////////////////////////////////////////////////////
    //
    // Other public functions that might be of interest
    //
    ///////////////////////////////////////////////////////


    public void setPipeline(int which) {
        if (which != pipeline_)
        {
            nt_.getEntry(PipelineKeyName).setNumber(which) ;
            pipeline_ = which ;
        }
    }

    public Pose3d getBotPose() {
        return botpose_ ;
    }

    public Pose3d getRedBotPose() {
        return wpired_;
    }

    public Pose3d getBlueBotPose() {
        return wpiblue_;
    }

    public int getId() {
        return id_ ;
    }

    public double getTL() {
        return tl_ ;
    }

    public double getCL() {
        return cl_ ;
    }

    public double getTS() {
        return ts_ ;
    }

    public boolean validTargets() {
        return valid_targets_ ;
    }

    public boolean isLimelightFound() {
        return found_ ;
    }

    public Retro[] getRetroData() {
        return retro_ ;
    }

    public Fiducial[] getFiducialData() {
        return fuds_ ;
    }

    public Detector[] getDetectorData() {
        return detectors_ ;
    }

    public Classifier[] getClassifierData() {
        return classifiers_ ;
    }

    @Override
    public String getStatus() {
        String st = "" ;

        if (!found_) {
            st = "<B>Limelight device not found</B>" ;
        }
        else {
            if (!valid_targets_) {
                st = "<B>No targets detected</B>" ;
            }
            else {
                st += getRetroStatus() ;
                st += getFiducialStatus() ;
                st += getClassifierStatus() ;
                st += getDetectorStatus() ;
            }
        }

        return st ;
    }

    public void retroComputeMyState() {
        if (cam_mode_ == CamMode.VisionProcessing)
        {
            if (nt_.containsKey("tv"))
            {
                double value = nt_.getEntry("tv").getNumber(0.0).doubleValue() ;
                if (value < 0.01)
                {
                    tv_ = false ;
                    tx_ = Double.MAX_VALUE;
                }
                else
                {
                    tv_ = true ;
                    tx_ = nt_.getEntry("tx").getNumber(0.0).doubleValue() ;
                    ty_ = nt_.getEntry("ty").getNumber(0.0).doubleValue() ;
                }
            }
            else {
                tv_ = false ;
                tx_ = Double.MAX_VALUE;
            }
        }
        else
        {
            tv_ = false ;
            tx_ = Double.MAX_VALUE;
        }

        putDashboard("ll-valid", DisplayType.Verbose, tv_);
        putDashboard("ll-yaw", DisplayType.Verbose, tx_) ;
    }

    @Override
    public void computeState() {
        MessageLogger logger = getRobot().getMessageLogger() ;
        String json = NetworkTableInstance.getDefault().getTable(getName()).getEntry("json").getString("") ;
        logger.startMessage(MessageType.Debug, getLoggerID()) ;
        if (json.length() == 0) {
            logger.add("<NULL>") ;
        }
        else {
            logger.add(json) ;
        }
        logger.endMessage() ;

        if (json.length() == 0) {
            found_ = false ;
        }
        else {
            found_ = true ;
            Object obj = JSONValue.parse(json);
            if (obj instanceof JSONObject) {
                parseLimelightJsonObject((JSONObject)obj) ;
            }
        }

        String str = "" ;
        if (fuds_ != null && fuds_.length > 0) {
            for(int i = 0 ; i < fuds_.length ; i++) {
                if (str.length() > 0) {
                    str += ", " ;
                }
                str += fuds_[i].id ;
            }
        }

        if (str == "")
            str = "NONE" ;

        logger.startMessage(MessageType.Debug, getLoggerID());
        logger.add("April Tags Seen: ", str) ;
        logger.endMessage();

        retroComputeMyState();
    }

    public double distantToTag(int id) {
        double ret = Double.MAX_VALUE ;

        if (fuds_ != null) {
            for(int i = 0 ; i < fuds_.length ; i++) {
                if (fuds_[i].id == id) {
                    ret = fuds_[i].targetToRobot.getTranslation().getNorm();
                }
            }
        }

        return ret;
    }

    public boolean hasAprilTag(int id) {
        boolean ret = false ;

        if (fuds_ != null) {
            for(int i = 0 ; i < fuds_.length ; i++) {
                if (fuds_[i].id == id) {
                    ret = true ;
                    break ;
                }
            }
        }

        return ret ;
    }

    private String getRetroStatus() {
        int count = 0 ;
        if (retro_ != null) {
            count = retro_.length ;
        }
        String st = "<br>Retro Targets, " + count + " detected<hr>" ;
        return st ;
    }

    private String pose3dToString(Pose3d p) {
        String st = p.getTranslation().toString() ;
        st += ", rx " + p.getRotation().getX() ;
        st += ", ry " + p.getRotation().getY() ;
        st += ", rz " + p.getRotation().getZ() ;
        return st ;
    }

    private String getOneFiducialStatus(Fiducial f) {
        String st = "" ;

        st += "<table border=\"1\">" ;
        st += "<tr><th>Item</th><th>Value</th></tr>" ;
        st += "<tr><td>ID</td><td>" + f.id + "</td></tr>" ;
        st += "<tr><td>Family</td><td>" + f.family + "</td></tr>" ;
        st += "<tr><td>ta</td><td>" + f.ta + "</td></tr>" ;
        st += "<tr><td>tx</td><td>" + f.tx + "</td></tr>" ;
        st += "<tr><td>txp</td><td>" + f.txp + "</td></tr>" ;
        st += "<tr><td>ty</td><td>" + f.ty + "</td></tr>" ;
        st += "<tr><td>typ</td><td>" + f.typ + "</td></tr>" ;
        st += "<tr><td>Cam-To-Target</td><td>" + pose3dToString(f.camToTarget) + "</td></tr>" ;
        st += "<tr><td>Robot-To-Field</td><td>" + pose3dToString(f.robotToField) + "</td></tr>" ;
        st += "<tr><td>Robot-To-Target</td><td>" + pose3dToString(f.robotToTarget) + "</td></tr>" ;
        st += "<tr><td>Target-To-Camera</td><td>" + pose3dToString(f.targetToCamera) + "</td></tr>" ;
        st += "<tr><td>Target-To-Robot</td><td>" + pose3dToString(f.targetToRobot) + "</td></tr>" ;

        st += "</table>";
        return st ;
    }

    private String getFiducialStatus() {
        int count ;

        if (fuds_ == null) {
            count = 0 ;
        }
        else {
            count = fuds_.length ;
        }
        String st = "<br>Fiducial Targets, " + count + " detected<hr>" ;

        if (count > 0) {
            for(int i = 0 ; i < fuds_.length ; i++) {
                st += getOneFiducialStatus(fuds_[i]) ;
            }
        }
        return st ;
    }

    private String getClassifierStatus() {
        int count ;

        if (classifiers_ == null) {
            count = 0 ;
        }
        else {
            count = classifiers_.length ;
        }
        String st = "<br>Classifier Targets, " + count + " detected<hr>" ;
        return st ;

    }

    private String getDetectorStatus() {
        int count ;

        if (detectors_ == null) {
            count = 0 ;
        }
        else {
            count = detectors_.length ;
        }
        String st = "<br>Detector Targets, " + count + " detected<hr>" ;
        return st ;
    }

    private String getStringFromObject(JSONObject obj, String name, String def) {
        String ret = def ;

        if (obj.containsKey(name)) {
            Object temp = obj.get(name) ;
            if (temp instanceof String) {
                ret = (String)temp ;
            }
        }

        return ret ;
    }

    private double getDoubleFromObject(JSONObject obj, String name, double def) {
        double ret = def ;

        if (obj.containsKey(name)) {
            Object temp = obj.get(name) ;
            if (temp instanceof Double) {
                ret = (Double)temp ;
            }
        }

        return ret ;
    }

    private int getIntFromObject(JSONObject obj, String name, int def) {
        int ret = def ;

        if (obj.containsKey(name)) {
            Object temp = obj.get(name) ;
            if (temp instanceof Double) {
                double d = (Double)temp ;
                ret = (int)d ;
            }
            else if (temp instanceof Long) {
                long l = (Long)temp ;
                ret = (int)l ;
            }
            else if (temp instanceof Integer) {
                ret = (Integer)temp ;
            }
        }

        return ret ;
    }

    private double [] convertToDoubleArray(JSONArray arr) {
        double [] data = new double[arr.size()] ;

        for(int i = 0 ; i < arr.size() ; i++) {
            if (!(arr.get(i) instanceof Double)) {
                return null ;
            }
            data[i] = (Double)arr.get(i) ;
        }

        return data ;
    }

    private Pose3d getPose3dFromObject(JSONObject obj, String name, Pose3d def) {
        Pose3d ret = def ;

        if (obj.containsKey(name)) {
            Object temp = obj.get(name) ;
            if (temp instanceof JSONArray) {
                JSONArray dataarr = (JSONArray)temp ;
                if (dataarr.size() == 6) {
                    double [] data = convertToDoubleArray(dataarr) ;
                    Translation3d trans = new Translation3d(data[0], data[1], data[2]) ;
                    Rotation3d rot = new Rotation3d(Math.toRadians(data[3]), Math.toRadians(data[4]), Math.toRadians(data[5])) ;
                    ret = new Pose3d(trans, rot) ;
                }
            }       
        } 

        return ret ;
    }

    private Translation2d[] getPointArrayFromObject(JSONObject obj, String name, Translation2d[] def) {
        Translation2d[] ret = def ;

        if (obj.containsKey(name)) {
            Object temp = obj.get(name) ;
            if (temp instanceof JSONArray) {
                double [] data = convertToDoubleArray((JSONArray)temp) ;
                if (data.length % 2 == 0) {
                    ret = new Translation2d[data.length / 2] ;
                    for(int i = 0 ;i < data.length ; i += 2) {
                        ret[i / 2] = new Translation2d(data[i], data[i + 1]) ;
                    }
                }
            }       
        } 

        return ret ;
    }

    private void parseClassifier(JSONArray entries) {
        if (entries.size() == 0) {
            classifiers_ = null  ;
        }
        else {
            classifiers_ = new Classifier[entries.size()] ;

            for(int i = 0 ; i < entries.size() ; i++) {
                Object temp = entries.get(i) ;
                if (!(temp instanceof JSONObject)) {
                    continue ;
                }

                JSONObject cobj = (JSONObject)temp ;
                Classifier c = new Classifier() ;

                c.className = getStringFromObject(cobj, "class", "") ;
                c.classId = getIntFromObject(cobj, "classID", -1) ;
                c.confidence = getDoubleFromObject(cobj, "conf", 0.0) ;

                classifiers_[i] = c ;
            }
        }
    }

    private void parseDetector(JSONArray entries) {
        if (entries.size() == 0) {
            detectors_ = null  ;
        }
        else {
            detectors_ = new Detector[entries.size()] ;

            for(int i = 0 ; i < entries.size() ; i++) {
                Object temp = entries.get(i) ;
                if (!(temp instanceof JSONObject)) {
                    continue ;
                }

                JSONObject dobj = (JSONObject)temp ;
                Detector d = new Detector() ;
    
                d.className = getStringFromObject(dobj, "class", "") ;
                d.classId = getIntFromObject(dobj, "classID", -1) ;
                d.confidence = getDoubleFromObject(dobj, "conf", 0.0) ;
                d.pts = getPointArrayFromObject(dobj, "pts", null) ;
                d.ta = getDoubleFromObject(dobj, "ta", 0.0) ;
                d.tx = getDoubleFromObject(dobj, "tx", 0.0) ;
                d.txp = getDoubleFromObject(dobj, "txp", 0.0) ;
                d.ty = getDoubleFromObject(dobj, "ty", 0.0) ;
                d.typ = getDoubleFromObject(dobj, "typ", 0.0) ;

                detectors_[i] = d ;
            }
        }
    }

    private void parseFiducials(JSONArray entries) {
        if (entries.size() == 0) {
            fuds_ = null ;
        }
        else {
            fuds_ = new Fiducial[entries.size()] ;

            for(int i = 0 ; i < entries.size() ; i++) {
                Object temp = entries.get(i) ;
                if (!(temp instanceof JSONObject)) {
                    continue ;
                }

                JSONObject fud = (JSONObject)temp ;
                Fiducial f = new Fiducial() ;
                f.id = getIntFromObject(fud, "fID", -1) ;
                f.family = getStringFromObject(fud, "fam", "") ;
                f.camToTarget = getPose3dFromObject(fud, "t6c_ts", null) ;
                f.robotToField = getPose3dFromObject(fud, "t6r_fs", null) ;
                f.robotToTarget = getPose3dFromObject(fud, "t6r_ts", null) ;
                f.targetToCamera = getPose3dFromObject(fud, "t6t_cs", null) ;
                f.targetToRobot = getPose3dFromObject(fud, "t6t_rs", null) ;
                f.ta = getDoubleFromObject(fud, "ta", 0.0) ;
                f.tx = getDoubleFromObject(fud, "tx", 0.0) ;
                f.txp = getDoubleFromObject(fud, "txp", 0.0) ;
                f.ty = getDoubleFromObject(fud, "ty", 0.0) ;
                f.typ = getDoubleFromObject(fud, "typ", 0.0) ;

                fuds_[i] = f ;
            }
        }
    }

    private void parseRetro(JSONArray entries) {
        if (entries.size() == 0) {
            retro_ = null ;
        }
        else {
            retro_ = new Retro[entries.size()] ;

            for(int i = 0 ; i < entries.size() ; i++) {
                Object temp = entries.get(i) ;
                if (!(temp instanceof JSONObject)) {
                    continue ;
                }

                JSONObject robj = (JSONObject)temp ;
                Retro r = new Retro() ;

                r.pts = getPointArrayFromObject(robj, "pts", null) ;
                r.camToTarget = getPose3dFromObject(robj, "t6c_ts", null) ;
                r.robotToField = getPose3dFromObject(robj, "t6r_fs", null) ;
                r.robotToTarget = getPose3dFromObject(robj, "t6r_ts", null) ;
                r.targetToCamera = getPose3dFromObject(robj, "t6t_cs", null) ;
                r.targetToRobot = getPose3dFromObject(robj, "t6t_rs", null) ;
                r.ta = getDoubleFromObject(robj, "ta", 0.0) ;
                r.tx = getDoubleFromObject(robj, "tx", 0.0) ;
                r.txp = getDoubleFromObject(robj, "txp", 0.0) ;
                r.ty = getDoubleFromObject(robj, "ty", 0.0) ;
                r.typ = getDoubleFromObject(robj, "typ", 0.0) ;

                retro_[i] = r ;
            }
        }
    }

    private void parseLimelightJsonObject(JSONObject tobj) {
        Object temp ;

        temp = tobj.get("Results") ;
        if (!(temp instanceof JSONObject)) {
            found_ = false ;
            return ;
        }

        JSONObject obj = (JSONObject)temp ;
        botpose_ = getPose3dFromObject(obj, "botpose", new Pose3d());
        wpired_ = getPose3dFromObject(obj, "botpose_wpired", new Pose3d());
        wpiblue_ = getPose3dFromObject(obj, "botpose_wpiblue", new Pose3d());
        id_ = getIntFromObject(obj, "pID", 0) ;
        tl_ = getDoubleFromObject(obj, "tl", 0.0) ;
        cl_ = getDoubleFromObject(obj, "cl", 0.0) ;
        ts_ = getDoubleFromObject(obj, "ts", 0.0) ;
        int v = getIntFromObject(obj, "v", -1) ;
        if (v == 0) {
            valid_targets_ = false ;
            return ;
        }
        else if (v == 1) {
            valid_targets_ = true ;
        }
        else if (v == -1) {
            valid_targets_ = false ;
            found_ = false ;
            return ;
        }
    
        temp = obj.get("Classifier") ;
        if (temp instanceof JSONArray) {
            parseClassifier((JSONArray)temp) ;
        }
        else {
            classifiers_ = null ;
        }

        temp = obj.get("Detector") ;
        if (temp instanceof JSONArray) {
            parseDetector((JSONArray)temp) ;
        }
        else {
            detectors_ = null ;
        }

        temp = obj.get("Fiducial") ;
        if (temp instanceof JSONArray) {
            parseFiducials((JSONArray)temp);
        }        
        else {
            fuds_ = null ;
        }

        temp = obj.get("Retro") ;
        if (temp instanceof JSONArray) {
            parseRetro((JSONArray)temp) ;
        }
        else {
            retro_ = null ;
        }
    }
}
