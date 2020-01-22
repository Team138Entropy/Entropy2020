package frc.robot.util.geometry;

public interface ICurvature<S> extends State<S> {
  double getCurvature();

  double getDCurvatureDs();
}
