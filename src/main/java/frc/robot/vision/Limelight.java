package frc.robot.vision;

import edu.wpi.first.networktables.*;


public class Limelight {

    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;
    private NetworkTableEntry light;
    private NetworkTable table;

    public Limelight() {

        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.ta = table.getEntry("ta");
        this.tv = table.getEntry("tv");
        this.light = table.getEntry("ledMode");
    }

    public TargetInfo GetTargetInfo() {
        var targetInfo = new TargetInfo();        
        targetInfo.X = this.tx.getDouble(0);
        targetInfo.Y = this.ty.getDouble(0);
        targetInfo.Area = this.ta.getDouble(0);
        targetInfo.HasTarget = this.tv.getDouble(0) == 0 ? false : true;
        return targetInfo;
    }

    public void TurnLightOff()
    {
        this.light.setNumber(1);
    }

    public void TrunLightOn() 
    {
        this.light.setNumber(3);
    }
}

