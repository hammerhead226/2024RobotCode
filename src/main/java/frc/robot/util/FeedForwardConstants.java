package frc.robot.util;

public class FeedForwardConstants {
  public final double ks;
  public final double kg;
  public final double kv;
  public final double ka;

  public FeedForwardConstants(double ks, double kg, double kv, double ka) {
    this.ks = ks;
    this.kg = kg;
    this.kv = kv;
    this.ka = ka;
  }

  public FeedForwardConstants(double kg) {
    this(0, kg, 0, 0);
  }

  public FeedForwardConstants(double ks, double kv) {
    this(ks, 0, kv, 0);
  }

  public FeedForwardConstants(double ks, double kg, double kv) {
    this(ks, kg, kv, 0);
  }
}
