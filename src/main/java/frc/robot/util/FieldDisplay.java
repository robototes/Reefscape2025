package frc.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class FieldDisplay {
  private static final Field2d m_field = new Field2d();
  public static ShuffleboardTab field = Shuffleboard.getTab("Field View");

  public static void initShuffleBoard() {

    field
        .add("Field", m_field)
        .withWidget(BuiltInWidgets.kField)
        .withPosition(0, 0)
        .withSize(13, 4);
  }
}
