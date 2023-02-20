package frc.robot.subsystems.toplevel;

public class RobotOperation {
    public enum Slot {
        LEFT,
        MIDDLE,
        RIGHT
    }
    
    public enum Height {
        TOP,
        MIDDLE,
        BOTTOM
    }
    
    public enum Piece{
        CONE,
        CUBE
    }

    public enum Action{
        PLACE,
        COLLECT
    }

    private Action action;
    private boolean auto;
    private Piece piece_type;
    private int tag_number;
    private Slot slot;
    private Height height;

    public RobotOperation() {}

    public RobotOperation(Action operation, boolean auto, Piece piece_type, int tag_number, Slot slot, Height height) {
        this.action = operation;
        this.auto = auto;
        this.piece_type = piece_type;
        this.tag_number = tag_number;
        this.slot = slot;
        this.height = height;
    }
    
    public Action getAction() {
        return action;
    }

    public void setAction(Action operation) {
        this.action = operation;
    }

    public boolean isAuto() {
        return auto;
    }

    public void setAuto(boolean auto) {
        this.auto = auto;
    }

    public Piece getPiece_type() {
        return piece_type;
    }

    public void setPiece_type(Piece piece_type) {
        this.piece_type = piece_type;
    }

    public int getTag_number() {
        return tag_number;
    }

    public void setTag_number(int tag_number) {
        this.tag_number = tag_number;
    }

    public Slot getSlot() {
        return slot;
    }

    public void setSlot(Slot slot) {
        this.slot = slot;
    }

    public Height getHeight() {
        return height;
    }

    public void setHeight(Height height) {
        this.height = height;
    }


}