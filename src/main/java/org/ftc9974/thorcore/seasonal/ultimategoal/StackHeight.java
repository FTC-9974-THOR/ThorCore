package org.ftc9974.thorcore.seasonal.ultimategoal;

public enum StackHeight {
    FOUR,
    ONE,
    ZERO;

    @Override
    public String toString() {
        switch (this) {
            case FOUR:
                return "Four";
            case ONE:
                return "One";
            case ZERO:
                return "Zero";
            default:
                return "Unknown";
        }
    }
}
