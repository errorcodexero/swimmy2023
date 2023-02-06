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
    }

    private Action action_ ;
    private GamePiece gp_ ;
    private int tag_ ;
    private Slot slot_ ;
    private Location loc_ ;

    public RobotOperation() {
        action_ = Action.Collect ;
        gp_ = GamePiece.None ;
        tag_ = 0 ;
        slot_ = Slot.Right;
        loc_ = Location.Middle ;
    }

    public RobotOperation(Action a, GamePiece gp, int t, Slot s, Location l) {
        action_ = a ;
        gp_ = gp;
        tag_ = t ;
        slot_ = s ;
        loc_ = l ;
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

    public int getAprilTag() {
        return tag_ ;
    }

    public void setAprilTag(int t) {
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
}
