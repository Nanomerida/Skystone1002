package org.firstinspires.ftc.teamcode.Mecanum.CRPosition;

public class Movement {
  
  ExternalEncoder left_y_encoder;
  ExternalEncoder right_y_encoder;
  ExternalEncoder x_encoder;
  
  
  public Movement(ExternalEncoder ly, ExternalEncoder ry, ExternalEnocoder x) {
    this.left_y_encoder = ly;
    this.right_y_encoder = ry;
    this.x_encoder = x;
  }
}
