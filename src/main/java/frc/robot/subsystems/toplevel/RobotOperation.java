package frc.robot.subsystems.toplevel;

public class RobotOperation {
    public enum Action
    {
        Collect,
        Place
    } ;

    public enum Slot
    {
        Left,
        Middle,
        Right
    } ;

    public enum Location
    {
        Bottom,
        Middle,
        Top
    } ;

    public enum GamePiece {
        Cube,
        Cone,
        None
    } ;

    public enum GridTagPosition {
        Left,
        Middle,
        Right
    }

    private Action action_ ;
    private GamePiece gp_ ;

    //
    // This is the tag we are interested in independent of the alliance color.
    // Tag 0 is always closest to the middle of the field.  Tag 1 is always the
    // cooperation grid.  Tag 2 is the grid closest to the edge of the field.
    // 
    private GridTagPosition tag_ ;
    private Slot slot_ ;
    private Location loc_ ;
    private boolean manual_ ;
    private boolean ground_ ;

    public RobotOperation() {
        action_ = Action.Collect ;
        gp_ = GamePiece.None ;
        tag_ = GridTagPosition.Middle;
        slot_ = Slot.Right;
        loc_ = Location.Middle ;
        manual_ = false ;
        ground_ = false ;
    }

    public RobotOperation(RobotOperation oper) {
        action_ = oper.getAction();
        gp_ = oper.getGamePiece() ;
        tag_ = oper.getAprilTag();
        slot_ = oper.getSlot();
        loc_ = oper.getLocation() ;
        manual_ = oper.getManual() ;
        ground_ = oper.getGround();
    }

    public RobotOperation(Action a, GamePiece gp, GridTagPosition t, Slot s, Location l, boolean m, boolean g) {
        action_ = a ;
        gp_ = gp;
        tag_ = t ;
        slot_ = s ;
        loc_ = l ;
        manual_ = m ;
        ground_ = g ;
    }

    public boolean getManual() {
        return manual_ ;
    }

    public void setManual(boolean v) {
        manual_ = v ;
    }

    public boolean getGround() {
        return ground_ ;
    }

    public void setGround(boolean v) {
        ground_ = v ;
    }

    public Action getAction() {
        return action_ ;
    }

    public void setAction(Action a) {
        action_ = a ;
    }

    public GamePiece getGamePiece() {
        return gp_ ;
    }

    public void setGamePiece(GamePiece gp) {
        gp_ = gp ;
    }

    public GridTagPosition getAprilTag() {
        return tag_ ;
    }

    public void setAprilTag(GridTagPosition t) {
        tag_ = t ;
    }

    public Slot getSlot() {
        return slot_ ;
    }

    public void setSlot(Slot s) {
        slot_ = s ;
    }

    public Location getLocation() {
        return loc_ ;
    }

    public void setLocation(Location l) {
        loc_ = l ;
    }

    public String toString() {
        String ret = "[" ;
        if (manual_) {
            ret += "manual";
        }
        else {
            ret += "auto" ;
        }
        ret += ", " + action_.toString() ;
        ret += ", " + gp_.toString();
        ret += ", Tag " + tag_ ;
        ret += ", Slot " + slot_.toString();
        ret += ", Loc " + loc_.toString();
        ret += ", " + (ground_ ? "ground" : "shelf");
        ret += "]" ;
        return ret ;
    }

    public boolean equals(RobotOperation oper) {
        if (action_ != oper.getAction())
            return false ;

        if (gp_ != oper.getGamePiece())
            return false ;

        if (tag_ != oper.getAprilTag())
            return false ;

        if (slot_ != oper.getSlot())
            return false ;

        if (loc_ != oper.getLocation())
            return false ;

        if (manual_ != oper.getManual())
            return false ;

        if (ground_ != oper.getGround())
            return false ;

        return true ;
    }
}
