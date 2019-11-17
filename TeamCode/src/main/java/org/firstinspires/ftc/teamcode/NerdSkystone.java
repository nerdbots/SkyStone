package org.firstinspires.ftc.teamcode;

public class NerdSkystone {
    private int priority;
    private double x_offset;
    private int position;
    private boolean isSkystone;


    public NerdSkystone(int position, double x_offset){
        this.position=position;
        this.x_offset=x_offset;
    }

    public void setPriority (int priority) {
        this.priority = priority;
    }
    public int getPriority() {
        return this.priority;
    }
    public void setX_offset (double x_offset){
        this.x_offset = x_offset;
    }
    public double getX_offset() {
        return this.x_offset;
    }
    public void setPosition (int position) {
        this.position = position;
    }
    public int getPosition() {
        return this.position;
    }
    public void setisSkystone (boolean isSkystone) {
        this.isSkystone = isSkystone;
    }
    public boolean getisSkystone() {
        return this.isSkystone;
    }
}
