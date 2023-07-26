package frc.robot.utils.sim.state;

public interface State<T> {
    boolean update();

    T get();

    void set(final T t);
}
