package frc.robot.commands.pathfinding;

import java.util.List;

public interface Node<T> extends Comparable<Node<T>> {
    Node<T> getPrev();
    void setPrev(final Node<T> node);

    double getG();
    void setG(final double g);
    double getF();
    void setF(final double f);

    default void setF(final Node<T> other) {
        setF(heuristicValue(other));
    }

    List<Node<T>> getNeighbors();
    void addNeighbors(final List<Node<T>> neighbors);

    T getValue();
    int getId();

    double heuristicValue(final Node<T> other);

    boolean equals(final Object other);
}
