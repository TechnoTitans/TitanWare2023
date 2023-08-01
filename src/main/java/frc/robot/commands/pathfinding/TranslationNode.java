package frc.robot.commands.pathfinding;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class TranslationNode implements Node<Translation2d> {
    private final Translation2d translation;
    private final List<Node<Translation2d>> neighbors;
    private Node<Translation2d> prev;

    private double g = Double.MAX_VALUE;
    private double f = Double.MAX_VALUE;

    public TranslationNode(
            final Translation2d translation,
            final TranslationNode prev,
            final List<Node<Translation2d>> neighbors
    ) {
        this.translation = translation;
        this.prev = prev;
        this.neighbors = neighbors;
    }

    public TranslationNode(final Translation2d translation) {
        this(translation, null, new ArrayList<>());
    }

    @Override
    public Node<Translation2d> getPrev() {
        return prev;
    }

    @Override
    public void setPrev(final Node<Translation2d> prev) {
        this.prev = prev;
    }

    @Override
    public double getG() {
        return g;
    }

    @Override
    public void setG(final double g) {
        this.g = g;
    }

    @Override
    public double getF() {
        return f;
    }

    @Override
    public void setF(double f) {
        this.f = f;
    }

    @Override
    public int compareTo(final Node<Translation2d> o) {
        return Double.compare(this.f, o.getF());
    }

    @Override
    public List<Node<Translation2d>> getNeighbors() {
        return neighbors;
    }

    @Override
    public void addNeighbors(final List<Node<Translation2d>> neighbors) {
        this.neighbors.addAll(neighbors);
    }

    @Override
    public Translation2d getValue() {
        return translation;
    }

    @Override
    public int getId() {
        return Objects.hash(translation);
    }

    @Override
    public boolean equals(final Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        final TranslationNode translationNode = (TranslationNode) o;
        return Objects.equals(translation, translationNode.translation);
    }

    @Override
    public int hashCode() {
        return getId();
    }

    @Override
    public double heuristicValue(final Node<Translation2d> other) {
        // TODO: is euclidean distance a good enough representation of the heuristic?
        final Translation2d now = getValue();
        final Translation2d next = other.getValue();

        final double xNow = now.getX();
        final double yNow = now.getY();
        final double xNext = next.getX();
        final double yNext = next.getY();

        // TODO: does this work?
        return now.getDistance(next);
    }
}
