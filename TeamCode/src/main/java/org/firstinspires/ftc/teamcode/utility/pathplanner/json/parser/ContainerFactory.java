package org.firstinspires.ftc.teamcode.utility.pathplanner.json.parser;

import java.util.List;
import java.util.Map;

public interface ContainerFactory {
  Map createObjectContainer();

  List creatArrayContainer();
}
