package org.ftc9974.thorcore.seasonal.ultimategoal;

/**
 * Represents the different heights of ring stacks in the Ultimate Goal autonomous period.
 */
public enum StackHeight {
    /**
     * Four rings
     */
    FOUR,
    /**
     * One ring
     */
    ONE,
    /**
     * Zero rings
     */
    ZERO;

    /**
     * Returns the name of this StackHeight.
     *
     * @return name
     */
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
