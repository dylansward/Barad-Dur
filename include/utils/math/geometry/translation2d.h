#pragma once

// These are required for Eigen to compile
// https://www.vexforum.com/t/eigen-integration-issue/61474/5
#undef __ARM_NEON__
#undef __ARM_NEON
#include <Eigen/Dense>


/**
 * Class representing a point in 2d space with x and y coordinates.
 * 
 * Assumes conventional cartesian coordinate system:
 * Looking down at the coordinate plane,
 * +X is right
 * +Y is up
 * +Theta is counterclockwise
 */
class Translation2d {
    public:
    /**
     * Constructs a Translation2d with the given x and y values.
     * 
     * @param x The x component of the translation.
     * @param y The y component of the translation.
     */
    Translation2d(const double& x, const double& y);

    /**
     * Constructs a Translation2d with the values from the given vector.
     * 
     * @param vector The vector whose values will be used.
     */
    Translation2d(const Eigen::Vector2d& vector);

    /**
     * Constructs a Translation2d given polar coordinates of the form (r, theta).
     * 
     * @param r The radius (magnitude) of the vector.
     * @param theta The angle (direction) of the vector.
     */
    Translation2d(const double& r, const Rotation2d& theta)

    /**
     * Returns the x value of the translation.
     * 
     * @return the x value of the translation.
     */
    double x();

    /**
     * Returns the y value of the translation.
     * 
     * @return the y value of the translation.
     */
    double y();

    /**
     * Returns the angle of the translation.
     * 
     * @return the angle of the translation.
     */
    Rotation2d theta();

    /**
     * Returns the vector as an Eigen::Vector2d.
     * 
     * @return Eigen::Vector2d with the same values as the translation.
     */
    Eigen::Vector2d as_vector();

    /**
     * Returns the norm/radius/magnitude/distance from origin.
     * 
     * @return the norm of the translation.
     */
    double norm();

    /**
     * Returns the distance between two translations.
     * 
     * @return the distance between two translations.
     */
    double distance(const Translation2d& other);

    /**
     * Applies a rotation to this translation around the origin.
     * 
     * Equivalent to multiplying a vector by a rotation matrix:
     * x = [cos, -sin][x]
     * y = [sin,  cos][y]
     * 
     * @param rotation the angle amount the translation will be rotated.
     * 
     * @return the new translation that has been rotated around the origin.
     */
    Translation2d rotate_by(const Rotation2d& rotation);

    /**
     * Applies a rotation to this translation around another given point.
     * 
     * [x] = [cos, -sin][x - otherx] + [otherx]
     * [y] = [sin,  cos][y - othery] + [othery]
     * 
     * @param other the center of rotation.
     * @param rotation the angle amount the translation will be rotated.
     * 
     * @return the translation that has been rotated.
     */
    Translation2d rotate_around(const Translation2d& other, const Rotation2d& rotation);

    /**
     * Calculates the mean of the translations in the list.
     * 
     * @param list std::vector containing a list of translations.
     * 
     * @return the single translation mean of the list of translations.
     */
    Translation2d mean(const std::vector<Translation2d>& list)

    /**
     * Returns the sum of two translations.
     * 
     * [x] = [x] + [otherx];
     * [y] = [y] + [othery];
     * 
     * @param other the other translation to add to this translation.
     * 
     * @return the sum of the two translations.
     */
    Translation2d operator+(const Translation2d& other);

    /**
     * Returns the difference of two translations.
     * 
     * [x] = [x] - [otherx]
     * [y] = [y] - [othery]
     * 
     * @param other the translation to subtract from this translation.
     * 
     * @return the difference of the two translations.
     */
    Translation2d operator-(const Translation2d& other);

    /**
     * Returns the inverse of this translation.
     * Equivalent to flipping the vector across the origin.
     * 
     * [x] = [-x]
     * [y] = [-y]
     * 
     * @return the inverse of this translation.
     */
    Translation2d operator-();

    /**
     * Returns this translation multiplied by a scalar.
     * 
     * [x] = [x] * [scalar]
     * [y] = [y] * [scalar]
     * 
     * @param scalar the scalar to multiply by.
     * 
     * @return this translation multiplied by a scalar.
     */
    Translation2d operator*(const double& scalar);

    /**
     * Returns this translation divided by a scalar.
     * 
     * [x] = [x] / [scalar]
     * [y] = [y] / [scalar]
     * 
     * @param scalar the scalar to divide by.
     * 
     * @return this translation divided by a scalar.
     */
    Translation2d operator/(const double& scalar);

    /**
     * Returns the dot product of two translations.
     * 
     * [scalar] = [x][otherx] + [y][othery]
     * 
     * @param other the other translation to find the dot product with.
     * 
     * @return the scalar valued dot product.
     */
    double operator*(const Translation2d& other);

    /**
     * Compares two translations.
     * Returns true if their components are each within 1e-9, to account for floating point error.
     * 
     * @param other the translation to compare to.
     * 
     * @return whether the two translations are equal.
     */
    bool operator==(const Translation2d& other);

    private:
    double m_x;
    double m_y;
};
