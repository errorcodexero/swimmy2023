package org.xero1425.base.subsystems.vision;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;
import org.xero1425.misc.MessageLogger ;
import org.xero1425.misc.MessageType ;
import org.xero1425.base.IVisionLocalization;
import org.xero1425.base.subsystems.Subsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLightSubsystem extends Subsystem implements IVisionLocalization {
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
    private double ts_ ;
    private boolean valid_targets_ ;

    private Retro [] retro_ ;
    private Fiducial[] fuds_ ;
    private Detector[] detectors_ ;
    private Classifier[] classifiers_ ;

    public LimeLightSubsystem(Subsystem parent, String name) {
        super(parent, name) ;
    }

    public LocationData getLocation() {
        LocationData ret = null ;

        if (found_ && valid_targets_ && fuds_ != null && fuds_.length > 0) {
            ret = new LocationData() ;
            ret.id = fuds_[0].id ;
            ret.location = fuds_[0].robotToField ;
            ret.type = LocationType.RobotFieldLocation ;
            ret.when = ts_ ;                                        // TODO is this right?  Only if they are using the new NT4 time syn functions
        }

        return ret ;
    }

    public int getId() {
        return id_ ;
    }

    public double getTL() {
        return tl_ ;
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
        String st ;

        if (p == null) {
            st = "<NULL>" ;
        }
        else {
            st = p.getTranslation().toString() ;
            st += ", rx " + p.getRotation().getX() ;
            st += ", ry " + p.getRotation().getY() ;
            st += ", rz " + p.getRotation().getZ() ;
        }
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
                    Rotation3d rot = new Rotation3d(data[3], data[4], data[5]) ;
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
        id_ = getIntFromObject(obj, "pID", 0) ;
        tl_ = getDoubleFromObject(obj, "tl", 10000.0) ;
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
