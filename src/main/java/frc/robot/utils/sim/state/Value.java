package frc.robot.utils.sim.state;

public class Value<T> implements State<T> {
    private boolean didUpdate;
    private T t;

    public Value(final T t) {
        this.t = t;
        this.didUpdate = true;
    }

    @Override
    public boolean update() {
        return didUpdate;
    }

    @Override
    public T get() {
        this.didUpdate = false;
        return t;
    }

    @Override
    public void set(final T t) {
        this.didUpdate = true;
        this.t = t;
    }
}