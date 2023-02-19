package frc.robot.subsystems.toplevel;

import java.util.HashMap;
import java.util.Map;

import org.json.simple.JSONObject;
import org.xero1425.misc.JsonReader;
import org.xero1425.misc.MessageLogger;
import org.xero1425.misc.MessageType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.toplevel.RobotOperation.GridTagPosition;
import frc.robot.subsystems.toplevel.RobotOperation.Slot;

public class FieldLocationData {

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

    public FieldLocationData(MessageLogger logger, String filename) throws Exception {
        logger_ = logger ;
        readLocationFile(filename) ;
    }

    public int getLoadingStationTag() {
        int ret = -1 ;

        Alliance a = DriverStation.getAlliance() ;
        if (a == Alliance.Red) {
            ret = 5 ;
        }
        else if (a == Alliance.Blue) {
            ret = 4 ;
        }
        
        return ret ;
    }

    public int getGridTag(RobotOperation.GridTagPosition tag) {
        int ret = -1 ;

        Alliance a = DriverStation.getAlliance() ;
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

    private Items getItems() {
        Items ret = null ;

        Alliance a = DriverStation.getAlliance() ;
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

    public Pose2d getLoadingStationPose(RobotOperation.Slot slot) {
        Items items = getItems() ;
        return items.getLoadingStation().getPose(slot2Text(slot));
    }

    public Pose2d getGridPose(GridTagPosition tag, RobotOperation.Slot slot) {
        Items items = getItems();
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

        if (!grid.containsKey("left")) {
            throw new Exception("the 'grid' section is missing the 'left' key") ;
        }
        item = readFieldItem((JSONObject)grid.get("left"));
        items.setGridLeft(item);

        if (!grid.containsKey("middle")) {
            throw new Exception("the 'grid' section is missing the 'middle' key") ;
        }
        item = readFieldItem((JSONObject)grid.get("middle"));
        items.setGridMiddle(item);

        if (!grid.containsKey("right")) {
            throw new Exception("the 'grid' section is missing the 'right' key") ;
        }
        item = readFieldItem((JSONObject)grid.get("right"));
        items.setGridRight(item);

        if (!colorobj.containsKey("loading-station")) {
            throw new Exception("the top level color section is missing the 'loading-station' key") ;
        }

        item = readFieldItem((JSONObject)colorobj.get("loading-station"));
        items.setLoadingStation(item);
    }

    private FieldItem readFieldItem(JSONObject obj) throws Exception {
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
}
