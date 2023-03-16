package frc.robot.subsystems.toplevel;

import java.util.HashMap;
import java.util.Map;

import org.json.simple.JSONObject;
import org.xero1425.misc.JsonReader;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.toplevel.RobotOperation.GridTagPosition;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;

public class FieldLocationData {
    //
    // This is the offset from the from the loading station April tag, to the position
    // we want the robot to drive to.  This offset gets adjusted for the red vs blue end
    // of the field.
    //
    static final double LoadingStationXOffset = 0.670;

    //
    // This is the offset from the loading station April tag, to the position of we want the
    // robot to drive to.  This offset gets adjusted for the left versus right loading station
    // on either side of the April tag.
    //
    // According to the game manual, this shouild be 0.859964 but we have adjusted slightly based on
    // emperical experiments.
    //
    static final double LoadingStationYOffset = 0.920;

    //
    // This is the offset from the grid april tag to the position we want the robot to drive to.
    // This offset gets adjusted for the red vs blue end of the field.
    //
    static final double GridXOffset = 0.86;

    //
    // This is the offset from the grid april tag to the position we want the robot to drive to for
    // the columns on the left and right of the april tag.  This offset gets adjusted for the left column
    // versus the right column.
    //
    // According to the game manual, this shouild be 0.859964 but we have adjusted slightly based on
    // emperical experiments.
    //
    static final double GridYOffset = 0.59000;

    public class FieldItem
    {
        private int tag_ ;
        private Map<String, Pose2d> locations_ ;

        public FieldItem(int tag) {
            tag_ = tag ;
            locations_ = new HashMap<String, Pose2d>() ;
        }

        public int getTag() {
            return tag_ ;
        }

        public void addPose(String name, Pose2d pose) {
            locations_.put(name, pose);
        }

        public Pose2d getPose(String name) {
            return locations_.get(name);
        }
    }

    public class Items {
        private FieldItem grid_left_ ;
        private FieldItem grid_right_ ;
        private FieldItem grid_middle_ ;
        private FieldItem loading_station_ ;

        public Items() {
        }

        public void setGridLeft(FieldItem i) {
            grid_left_ = i ;
        }

        public FieldItem getGridLeft() {
            return grid_left_ ;
        }

        public void setGridRight(FieldItem i) {
            grid_right_ = i ;
        }

        public FieldItem getGridRight() {
            return grid_right_ ;
        }

        public void setGridMiddle(FieldItem i) {
            grid_middle_ = i ;
        }

        public FieldItem getGridMiddle() {
            return grid_middle_ ;
        }

        public void setLoadingStation(FieldItem i) {
            loading_station_ = i ;
        }

        public FieldItem getLoadingStation() {
            return loading_station_;
        }
    }

    private MessageLogger logger_ ;
    private Items red_items_ ;
    private Items blue_items_;
    private AprilTagFieldLayout layout_ ;
    private String data_type_ ;

    public FieldLocationData(MessageLogger logger, String filename, AprilTagFieldLayout layout) throws Exception {
        logger_ = logger ;
        layout_ = layout ;

        if (filename == null) {
            data_type_ = "Computed" ;
            computeLocationData();
        } else {
            data_type_ = "File" ;
            readLocationFile(filename) ;
        }
        dumpDataToLogger();
    }

    private void dumpDataToLogger() {
        logger_.startMessage(MessageType.Debug);
        logger_.add("================================================================================================================");
        logger_.endMessage();
        logger_.startMessage(MessageType.Debug);
        logger_.add("Data Type: ").add(data_type_);
        logger_.endMessage();
        dumpOneColor(Alliance.Red);
        dumpOneColor(Alliance.Blue);
        logger_.startMessage(MessageType.Debug);
        logger_.add("================================================================================================================");
        logger_.endMessage();
    }

    private Translation2d difference(Pose3d tag, Pose2d loc) {
        return tag.toPose2d().getTranslation().minus(loc.getTranslation()) ;
    }

    private void dumpOneColor(Alliance a) {
        Pose3d p3d ;
        Pose2d p2d;

        logger_.startMessage(MessageType.Debug);
        logger_.add("Alliance Color: ");
        logger_.add(a.toString());
        logger_.endMessage();

        p3d = layout_.getTagPose(getLoadingStationTag(a)).get() ;
        logger_.startMessage(MessageType.Debug);
        logger_.add("LoadingStation:");
        logger_.add("tag", getLoadingStationTag(a));
        logger_.add("position", p3d.getTranslation());
        logger_.endMessage();

        p2d = getLoadingStationPose(a, Slot.Left) ;
        logger_.startMessage(MessageType.Debug);
        logger_.add("    ");
        logger_.add("left", p2d);
        logger_.add("difference", difference(p3d, p2d));
        logger_.endMessage();

        p2d = getLoadingStationPose(a, Slot.Right) ;
        logger_.startMessage(MessageType.Debug);
        logger_.add("    ");
        logger_.add("right", p2d);
        logger_.add("difference", difference(p3d, p2d));
        logger_.endMessage();

        dumpGrid(a, GridTagPosition.Left);
        dumpGrid(a, GridTagPosition.Right);
        dumpGrid(a, GridTagPosition.Middle);
    }

    private void dumpGrid(Alliance a, GridTagPosition tag) {
        Pose3d p3d ;
        Pose2d p2d;

        p3d = layout_.getTagPose(getGridTag(a, tag)).get();

        logger_.startMessage(MessageType.Debug);
        logger_.add("Grid:");
        logger_.add("tagname", tag.toString());
        logger_.add("tag", getGridTag(a, tag));
        logger_.add("position", p3d.getTranslation());
        logger_.endMessage();

        p2d = getGridPose(a, tag, Slot.Left);
        logger_.startMessage(MessageType.Debug);
        logger_.add("    ");
        logger_.add("left", p2d);
        logger_.add("difference", difference(p3d, p2d));
        logger_.endMessage();

        p2d = getGridPose(a, tag, Slot.Middle);
        logger_.startMessage(MessageType.Debug);
        logger_.add("    ");
        logger_.add("middle", p2d);
        logger_.add("difference", difference(p3d, p2d));
        logger_.endMessage();

        p2d = getGridPose(a, tag, Slot.Right);
        logger_.startMessage(MessageType.Debug);
        logger_.add("    ");
        logger_.add("right", p2d);
        logger_.add("difference", difference(p3d, p2d));
        logger_.endMessage();
    }

    public int getLoadingStationTag(Alliance a) {
        int ret = -1 ;

        if (a == Alliance.Invalid)
            a = DriverStation.getAlliance() ;

        if (a == Alliance.Red) {
            ret = 5 ;
        }
        else if (a == Alliance.Blue) {
            ret = 4 ;
        }
        
        return ret ;
    }

    public int getGridTag(Alliance a, RobotOperation.GridTagPosition tag) {
        int ret = -1 ;

        if (a == Alliance.Invalid) {
            a = DriverStation.getAlliance() ;
        }

        if (a == Alliance.Red) {
            switch(tag) {
                case Left:
                    ret = 1 ;
                    break ;

                case Middle:
                    ret = 2 ;
                    break ;

                case Right:
                    ret = 3 ;
                    break ;
            }
        }
        else {
            switch(tag) {
                case Left:
                    ret = 6 ;
                    break ;

                case Middle:
                    ret = 7 ;
                    break ;

                case Right:
                    ret = 8 ;
                    break ;
            }
        }

        return ret;
    }

    private Items getItems(Alliance a) {
        Items ret = null ;

        if (a == Alliance.Invalid) {
            a = DriverStation.getAlliance() ;
        }

        if (a == Alliance.Red) {
            ret = red_items_ ;
        }
        else if (a == Alliance.Blue) {
            ret = blue_items_ ;
        }

        return ret ;
    }

    private String slot2Text(RobotOperation.Slot slot) {
        String which = "" ;
        if (slot == Slot.Left) {
            which = "left" ;
        }
        else if (slot == Slot.Right) {
            which = "right" ;
        }
        else {
            which = "middle" ;
        }

        return which;
    }

    public Pose2d getLoadingStationPose(Alliance a, RobotOperation.Slot slot) {
        Items items = getItems(a) ;
        return items.getLoadingStation().getPose(slot2Text(slot));
    }

    public Pose2d getGridPose(Alliance a, GridTagPosition tag, RobotOperation.Slot slot) {
        Items items = getItems(a);
        FieldItem gitem = null;

        switch(tag) {
            case Left:
                gitem = items.getGridLeft() ;
                break ;

            case Right:
                gitem = items.getGridRight() ;
                break ;
                
            case Middle:
                gitem = items.getGridMiddle() ;
                break ;                
        }

        return gitem.getPose(slot2Text(slot));
    }

    private void readLocationFile(String filename) throws Exception {
        logger_.startMessage(MessageType.Info);
        logger_.add("reading field location file ").addQuoted(filename) ;
        logger_.endMessage();    

        JSONObject contents = JsonReader.readFile(filename, logger_) ;

        if (!contents.containsKey("red")) {
            throw new Exception("the field location data file is invalid, missing top level 'red' section") ;
        }

        if (!contents.containsKey("blue")) {
            throw new Exception("the field location data file is invalid, missing top level 'blue' section") ;
        }

        red_items_ = new Items();
        blue_items_ = new Items();

        JSONObject redchild = (JSONObject)contents.get("red");
        readColorSection(red_items_, redchild);

        JSONObject bluechild = (JSONObject)contents.get("blue");
        readColorSection(blue_items_, bluechild);
    }

    private void readColorSection(Items items, JSONObject colorobj) throws Exception {
        FieldItem item ;

        if (!colorobj.containsKey("grid")) {
            throw new Exception("the top level color section is missing the 'grid' key") ;
        }

        JSONObject grid = (JSONObject)colorobj.get("grid") ;
        double yoffset = (Double)grid.get("yoffset");

        if (!grid.containsKey("left")) {
            throw new Exception("the 'grid' section is missing the 'left' key") ;
        }
        item = readFieldItem((JSONObject)grid.get("left"), false, yoffset);
        items.setGridLeft(item);

        if (!grid.containsKey("middle")) {
            throw new Exception("the 'grid' section is missing the 'middle' key") ;
        }
        item = readFieldItem((JSONObject)grid.get("middle"), false, yoffset);
        items.setGridMiddle(item);

        if (!grid.containsKey("right")) {
            throw new Exception("the 'grid' section is missing the 'right' key") ;
        }
        item = readFieldItem((JSONObject)grid.get("right"), false, yoffset);
        items.setGridRight(item);

        if (!colorobj.containsKey("loading-station")) {
            throw new Exception("the top level color section is missing the 'loading-station' key") ;
        }

        item = readFieldItem((JSONObject)colorobj.get("loading-station"), true, 0.0);
        items.setLoadingStation(item);
    }

    private FieldItem readFieldItem(JSONObject obj, boolean lstation, double yoffset) throws Exception {
        if (!obj.containsKey("grid-id")) {
            throw new Exception("Field Item is missing the 'grid-id' key");
        }

        Long gidobj = (Long)obj.get("grid-id");
        if (gidobj == null) {
            throw new Exception("Field Item has 'grid-id' but its not an integer");
        }

        FieldItem item = new FieldItem(gidobj.intValue());

        for(Object key : obj.keySet()) {
            String keystr = (String)key;
            if (keystr != null && !keystr.equals("grid-id")) {
                JSONObject poseobj = (JSONObject)obj.get(keystr);
                if (!poseobj.containsKey("x")) {
                    throw new Exception("Field Item '" + keystr + "' is missing an 'x' value");
                }
                if (!poseobj.containsKey("y")) {
                    throw new Exception("Field Item '" + keystr + "' is missing a 'y' value");
                }
                if (!poseobj.containsKey("heading")) {
                    throw new Exception("Field Item '" + keystr + "' is missing a 'heading' value");
                }

                double dx = getDouble(poseobj, "x") ;
                double dy = getDouble(poseobj, "y") ;
                double dheading = getDouble(poseobj, "heading") ;
                item.addPose(keystr, new Pose2d(dx, dy, Rotation2d.fromDegrees(dheading)));
            }
        }

        if (!lstation) {
            Pose2d middle = item.getPose("middle") ;
            if (middle == null) {
                throw new Exception("cannot read middle pose from location file");
            }

            double sign = 1.0 ;
            if (item.tag_ >= 6) {
                sign = -1.0 ;
            }

            if (item.getPose("left") == null) {
                Pose2d p = new Pose2d(middle.getX(), middle.getY() - sign * yoffset, middle.getRotation());
                item.addPose("left", p) ;
            }

            if (item.getPose("right") == null) {
                Pose2d p = new Pose2d(middle.getX(), middle.getY() + sign * yoffset, middle.getRotation());
                item.addPose("right", p) ;
            }
        }
        
        return item;
    }

    private double getDouble(JSONObject obj, String key) throws Exception {
        double ret ;

        Object vobj = obj.get(key);

        if (vobj instanceof Integer) {
            ret = ((Integer)vobj).intValue();
        }
        else if (vobj instanceof Long) {
            ret = ((Long)vobj).intValue();
        }
        else if (vobj instanceof Double) {
            ret = (Double)vobj;
        }
        else {
            throw new Exception("invalid value in field '" + key + "' in field location file");
        }


        return ret ;
    }

    private void computeLocationData() {
        red_items_ = compuateAlliance(Alliance.Red);
        blue_items_ = compuateAlliance(Alliance.Blue);
    }

    private Rotation2d rotate180(Rotation2d rot) {
        return rot.rotateBy(Rotation2d.fromDegrees(180.0));
    }

    private Items compuateAlliance(Alliance a) {
        Items ret = new Items();

        int tag ;
        Pose2d tagpose ;
        Pose2d p2d ;
        double sign = (a == Alliance.Red) ? 1.0 : -1.0 ;
        FieldItem fi ;

        tag = getLoadingStationTag(a);
        tagpose = layout_.getTagPose(tag).get().toPose2d();
        fi = new FieldItem(tag);
        p2d = new Pose2d(tagpose.getX() + sign * LoadingStationXOffset, tagpose.getY() - sign * LoadingStationYOffset, rotate180(tagpose.getRotation()));
        fi.addPose("left", p2d);
        p2d = new Pose2d(tagpose.getX() + sign * LoadingStationXOffset, tagpose.getY() + sign * LoadingStationYOffset, rotate180(tagpose.getRotation()));
        fi.addPose("right", p2d);
        ret.setLoadingStation(fi);

        tag = getGridTag(a, GridTagPosition.Left);
        tagpose = layout_.getTagPose(tag).get().toPose2d();
        fi = new FieldItem(tag);
        p2d = new Pose2d(tagpose.getX() - sign * GridXOffset, tagpose.getY() - sign * GridYOffset, rotate180(tagpose.getRotation()));
        fi.addPose("left", p2d);
        p2d = new Pose2d(tagpose.getX() - sign * GridXOffset, tagpose.getY(), rotate180(tagpose.getRotation()));
        fi.addPose("middle", p2d);
        p2d = new Pose2d(tagpose.getX() - sign * GridXOffset, tagpose.getY() + sign * GridYOffset, rotate180(tagpose.getRotation()));
        fi.addPose("right", p2d);
        ret.setGridLeft(fi);

        tag = getGridTag(a, GridTagPosition.Middle);
        tagpose = layout_.getTagPose(tag).get().toPose2d();
        fi = new FieldItem(tag);
        p2d = new Pose2d(tagpose.getX() - sign * GridXOffset, tagpose.getY() - sign * GridYOffset, rotate180(tagpose.getRotation()));
        fi.addPose("left", p2d);
        p2d = new Pose2d(tagpose.getX() - sign * GridXOffset, tagpose.getY(), rotate180(tagpose.getRotation()));
        fi.addPose("middle", p2d);
        p2d = new Pose2d(tagpose.getX() - sign * GridXOffset, tagpose.getY() + sign * GridYOffset, rotate180(tagpose.getRotation()));
        fi.addPose("right", p2d);
        ret.setGridMiddle(fi);

        tag = getGridTag(a, GridTagPosition.Right);
        tagpose = layout_.getTagPose(tag).get().toPose2d();
        fi = new FieldItem(tag);
        p2d = new Pose2d(tagpose.getX() - sign * GridXOffset, tagpose.getY() - sign * GridYOffset, rotate180(tagpose.getRotation()));
        fi.addPose("left", p2d);
        p2d = new Pose2d(tagpose.getX() - sign * GridXOffset, tagpose.getY(), rotate180(tagpose.getRotation()));
        fi.addPose("middle", p2d);
        p2d = new Pose2d(tagpose.getX() - sign * GridXOffset, tagpose.getY() + sign * GridYOffset, rotate180(tagpose.getRotation()));
        fi.addPose("right", p2d);
        ret.setGridRight(fi);

        return ret;
    }
}
