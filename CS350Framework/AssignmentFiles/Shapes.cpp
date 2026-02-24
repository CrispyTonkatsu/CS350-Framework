///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis, Edgar Jose Donoso Mansilla
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////

#include "Precompiled.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <numeric>

//-----------------------------------------------------------------------------LineSegment
LineSegment::LineSegment() { mStart = mEnd = Vector3::cZero; }

LineSegment::LineSegment(Math::Vec3Param start, Math::Vec3Param end)
{
    mStart = start;
    mEnd = end;
}

DebugShape& LineSegment::DebugDraw() const
{
    return gDebugDrawer->DrawLine(*this);
}

//-----------------------------------------------------------------------------Ray
Ray::Ray() { mStart = mDirection = Vector3::cZero; }

Ray::Ray(Math::Vec3Param start, Math::Vec3Param dir)
{
    mStart = start;
    mDirection = dir;
}

Ray Ray::Transform(const Matrix4& transform) const
{
    Ray transformedRay;
    transformedRay.mStart = Math::TransformPoint(transform, mStart);
    transformedRay.mDirection = Math::TransformNormal(transform, mDirection);
    return transformedRay;
}

Vector3 Ray::GetPoint(float t) const { return mStart + mDirection * t; }

DebugShape& Ray::DebugDraw(float t) const
{
    return gDebugDrawer->DrawRay(*this, t);
}

//-----------------------------------------------------------------------------PCA
// Helpers
Matrix3 ComputeCovarianceMatrix(const std::vector<Vector3>& points)
{
    const Vector3 mean{
    (1.f / static_cast<float>(points.size())) *
    std::accumulate(points.begin(), points.end(), Vector3()),
    };

    return {
    (1.f / static_cast<float>(points.size())) *
    std::accumulate(points.begin(), points.end(), Matrix3(), //
                    [mean](const Matrix3& acc, const Vector3& val)
                    {
                        const Vector3 v{val - mean};

                        const Matrix3 mat{
                        Math::Sq(v.x), v.x * v.y, v.x * v.z,
                        v.x * v.y, Math::Sq(v.y), v.y * v.z,
                        v.x * v.z, v.y * v.z, Math::Sq(v.z),
                        };

                        return acc + mat;
                    }),
    };
}

Matrix3 ComputeJacobiRotation(const Matrix3& matrix)
{
    // Compute the jacobi rotation matrix that will turn the largest (magnitude)
    // off-diagonal element of the input matrix into zero. Note: the input
    // matrix should always be (near) symmetric.

    // Getter to simplify the look of the method
    const auto get_index{[](size_t i, size_t j) { return i + (3 * j); }};

    float max_val{std::numeric_limits<float>::min()};
    size_t max_i{0};
    size_t max_j{0};

    for (size_t i{0}; i < 3; i++)
    {
        for (size_t j{i + 1}; j < 3; j++)
        {
            const float abs_val{Math::Abs(matrix.array[get_index(i, j)])};
            if (abs_val > max_val)
            {
                max_val = abs_val;
                max_i = i;
                max_j = j;
            }
        }
    }

    // Getting the local matrix abcd, b == c as it is near symmetric
    const float a{matrix.array[get_index(max_i, max_i)]};
    const float b{matrix.array[get_index(max_i, max_j)]};
    const float d{matrix.array[get_index(max_j, max_j)]};

    // Calculating beta and the tangents with the formulas from the slides
    const float beta{(d - a) / (2.f * b)};
    const float tangent{
    (Math::GetSign(beta)) / (Math::Abs(beta) + Math::Sqrt(Math::Sq(beta) + 1)),
    };

    // Using the formulas for the cosine and sine values from wikipedia
    const float cosine{1.f / (Math::Sqrt(Math::Sq(tangent) + 1))};
    const float sine{cosine * tangent};

    // Creating the rotation matrix
    Matrix3 output{Matrix3::cIdentity};
    output.array[get_index(max_i, max_i)] = cosine;
    output.array[get_index(max_i, max_j)] = sine;
    output.array[get_index(max_j, max_i)] = -sine;
    output.array[get_index(max_j, max_j)] = cosine;

    return output;
}

void ComputeEigenValuesAndVectors(const Matrix3& covariance,
                                  Vector3& eigenValues, Matrix3& eigenVectors,
                                  int maxIterations)
{
    // Iteratively rotate off the largest off-diagonal elements until the
    // resultant matrix is diagonal or maxIterations.

    const float epsilon{Math::DebugEpsilon()};

    const auto is_diagonal{
    [epsilon](const Matrix3& mat)
    {
        for (size_t i{0}; i < 3; i++)
        {
            for (size_t j{0}; j < 3; j++)
            {
                if (i == j)
                {
                    continue;
                }

                if (mat.array[i + (3 * j)] > epsilon)
                {
                    return false;
                }
            }
        }

        return true;
    },
    };

    Matrix3 diagonal_mat{covariance};
    eigenVectors = Matrix3::cIdentity;

    for (int i{0}; i < maxIterations; i++)
    {
        if (is_diagonal(diagonal_mat))
        {
            break;
        }

        const Matrix3 jacobi{ComputeJacobiRotation(covariance)};

        diagonal_mat = jacobi.Transposed() * diagonal_mat * jacobi;
        eigenVectors = eigenVectors * jacobi;
    }

    eigenValues.x = diagonal_mat.m00;
    eigenValues.y = diagonal_mat.m11;
    eigenValues.z = diagonal_mat.m22;
}

//-----------------------------------------------------------------------------Sphere
Sphere::Sphere()
{
    mCenter = Vector3::cZero;
    mRadius = 0;
}

Sphere::Sphere(const Vector3& center, float radius)
{
    mCenter = center;
    mRadius = radius;
}

/**
 * Method to expand a sphere with a given start start_center and start radius
 * @param points The points to expand the sphere on
 * @param start_center The starting center of the sphere
 * @param start_radius The starting radius of the sphere
 * @return pair of center and radius for the sphere
 */
std::pair<Vector3, float> sphere_expansion(
const std::vector<Vector3>& points,
const Vector3& start_center,
const float start_radius)
{
    Vector3 center{start_center};
    float radius{start_radius};

    for (Vector3 point : points)
    {
        const Vector3 to_point{(point - center)};
        const float distance{to_point.Length()};

        if (distance <= radius)
        {
            continue;
        }

        const Vector3 bound_point{
        center - ((radius / distance) * to_point),
        };

        center = 0.5f * (bound_point + point);
        radius = 0.5f * Math::Distance(bound_point, point);
    }

    // TODO: Left off here repairing the method
    return {center, radius};
}

void Sphere::ComputeCentroid(const std::vector<Vector3>& points)
{
    // The centroid method is roughly describe as: find the centroid (not mean)
    // of all points and then find the furthest away point from the centroid.

    const Vector3 max{
    std::accumulate(points.begin(), points.end(), Vector3(),
                    [](const Vector3& acc, const Vector3& val)
                    {
                        return Math::Max(acc, val);
                    }),
    };

    const Vector3 min{
    std::accumulate(points.begin(), points.end(), Vector3(),
                    [](const Vector3& acc, const Vector3& val)
                    {
                        return Math::Min(acc, val);
                    }),
    };

    const Vector3 centroid{0.5f * (max + min)};

    float max_distance{std::numeric_limits<float>::lowest()};
    for (Vector3 point : points)
    {
        const float distance{Math::Distance(centroid, point)};
        max_distance = Math::Max(distance, max_distance);
    }

    mRadius = max_distance;
    mCenter = centroid;
}

std::pair<Vector3, Vector3> find_min_max_on_axis(
const std::vector<Vector3>& points,
const Vector3& axis)
{
    const Vector3 max_point{
    *std::max_element(points.begin(), points.end(),
                      [axis](const Vector3& a, const Vector3& b)
                      {
                          return a.Dot(axis) < b.Dot(axis);
                      }),
    };

    const Vector3 min_point{
    *std::min_element(points.begin(), points.end(),
                      [axis](const Vector3& a, const Vector3& b)
                      {
                          return a.Dot(axis) < b.Dot(axis);
                      }),
    };

    return {min_point, max_point};
}

void Sphere::ComputeRitter(const std::vector<Vector3>& points)
{
    // The ritter method:
    // Find the largest spread on each axis.
    // Find which axis' pair of points are the furthest (Euclidean distance)
    // apart. Choose the center of this line as the sphere center. Now
    // incrementally expand the sphere.

    const std::array<std::pair<Vector3, Vector3>, 3> axis_extrema{
    find_min_max_on_axis(points, Vector3::cXAxis),
    find_min_max_on_axis(points, Vector3::cYAxis),
    find_min_max_on_axis(points, Vector3::cZAxis),
    };

    const std::pair<Vector3, Vector3> extrema_to_use{*std::max_element(
    axis_extrema.begin(),
    axis_extrema.end(),
    [](const std::pair<
           Vector3, Vector3>& a,
       const std::pair<
           Vector3, Vector3>& b)
    {
        return (a.first - a.second).
        LengthSq() < (b.first - b.
            second).LengthSq();
    })};

    const auto min{extrema_to_use.first};
    const auto max{extrema_to_use.second};

    const std::pair<Vector3, float> final_sphere{
    sphere_expansion(points, 0.5f * (min + max), 0.5f * (max - min).Length()),
    };

    mCenter = final_sphere.first;
    mRadius = final_sphere.second;
}

void Sphere::ComputePCA(const std::vector<Vector3>& points)
{
    // Compute the eigen values and vectors. Take the largest eigen vector as
    // the axis of largest spread. Compute the sphere center as the center of
    // this axis then expand by all points.

    const Matrix3 covariance{ComputeCovarianceMatrix(points)};

    Vector3 eigen_values{};
    Matrix3 eigen_vectors{};

    ComputeEigenValuesAndVectors(covariance, eigen_values, eigen_vectors, 1000);

    int max_spread_index{0};
    float max_eigen{eigen_values.x};

    for (int i{1}; i < 3; i++)
    {
        if (eigen_values.array[i] > max_eigen)
        {
            max_spread_index = i;
            max_eigen = eigen_values.array[i];
        }
    }

    const Vector3 axis{eigen_vectors.Basis(max_spread_index)};

    const std::pair<Vector3, Vector3> extrema{
    find_min_max_on_axis(points, axis)};

    const Vector3 min{extrema.first};
    const Vector3 max{extrema.second};

    const Vector3 center{
    0.5f * (min + max)
    };

    const float radius{
    0.5f * (max - min).Length()
    };

    const std::pair<Vector3, float> final_sphere{
    sphere_expansion(points, center, radius),
    };

    mCenter = final_sphere.first;
    mRadius = final_sphere.second;
}

bool Sphere::ContainsPoint(const Vector3& point)
{
    return PointSphere(point, mCenter, mRadius);
}

Vector3 Sphere::GetCenter() const { return mCenter; }

float Sphere::GetRadius() const { return mRadius; }

bool Sphere::Compare(const Sphere& rhs, float epsilon) const
{
    float posDiff = Math::Length(mCenter - rhs.mCenter);
    float radiusDiff = Math::Abs(mRadius - rhs.mRadius);

    return posDiff < epsilon && radiusDiff < epsilon;
}

DebugShape& Sphere::DebugDraw() const
{
    return gDebugDrawer->DrawSphere(*this);
}

//-----------------------------------------------------------------------------Aabb
Aabb::Aabb()
{
    // set the aabb to an initial bad value (where the min is smaller than the
    // max)
    mMin.Splat(Math::PositiveMax());
    mMax.Splat(-Math::PositiveMax());
}

Aabb::Aabb(const Vector3& min, const Vector3& max)
{
    mMin = min;
    mMax = max;
}

Aabb Aabb::BuildFromCenterAndHalfExtents(const Vector3& center,
                                         const Vector3& halfExtents)
{
    return Aabb(center - halfExtents, center + halfExtents);
}

float Aabb::GetVolume() const
{
    // Return the aabb's volume
    const Vector3 lengths{2.f * GetHalfSize()};
    return std::accumulate(lengths.array, lengths.array + 3, 1.f,
                           [](float acc, float val) { return acc * val; });
}

float Aabb::GetSurfaceArea() const
{
    // Return the aabb's surface area
    const Vector3 lengths{2.f * GetHalfSize()};
    return 2.f *
    ((lengths.x * lengths.y) + (lengths.x * lengths.z) +
        (lengths.y * lengths.z));
}

bool Aabb::Contains(const Aabb& aabb) const
{
    // Return if aabb is completely contained in this
    return PointAabb(aabb.mMin, mMin, mMax) && PointAabb(aabb.mMax, mMin, mMax);
}

void Aabb::Expand(const Vector3& point)
{
    for (uint32_t i = 0; i < 3; ++i)
    {
        mMin[i] = Math::Min(mMin[i], point[i]);
        mMax[i] = Math::Max(mMax[i], point[i]);
    }
}

Aabb Aabb::Combine(const Aabb& lhs, const Aabb& rhs)
{
    Aabb result;
    for (uint32_t i = 0; i < 3; ++i)
    {
        result.mMin[i] = Math::Min(lhs.mMin[i], rhs.mMin[i]);
        result.mMax[i] = Math::Max(lhs.mMax[i], rhs.mMax[i]);
    }
    return result;
}

bool Aabb::Compare(const Aabb& rhs, float epsilon) const
{
    float pos1Diff = Math::Length(mMin - rhs.mMin);
    float pos2Diff = Math::Length(mMax - rhs.mMax);

    return pos1Diff < epsilon && pos2Diff < epsilon;
}

void Aabb::Transform(const Vector3& scale, const Matrix3& rotation,
                     const Vector3& translation)
{
    /******Student:Assignment2******/
    // Compute aabb of this aabb after it is transformed.
    // You should use the optimize method discussed in class (not transforming
    // all 8 points).
    Warn("Assignment2: Required function un-implemented");
}

Vector3 Aabb::GetMin() const { return mMin; }

Vector3 Aabb::GetMax() const { return mMax; }

Vector3 Aabb::GetCenter() const { return (mMin + mMax) * 0.5f; }

Vector3 Aabb::GetHalfSize() const { return (mMax - mMin) * 0.5f; }

DebugShape& Aabb::DebugDraw() const { return gDebugDrawer->DrawAabb(*this); }

//-----------------------------------------------------------------------------Triangle
Triangle::Triangle() { mPoints[0] = mPoints[1] = mPoints[2] = Vector3::cZero; }

Triangle::Triangle(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
    mPoints[0] = p0;
    mPoints[1] = p1;
    mPoints[2] = p2;
}

DebugShape& Triangle::DebugDraw() const
{
    return gDebugDrawer->DrawTriangle(*this);
}

//-----------------------------------------------------------------------------Plane
Plane::Plane() { mData = Vector4::cZero; }

Plane::Plane(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
    Set(p0, p1, p2);
}

Plane::Plane(const Vector3& normal, const Vector3& point)
{
    Set(normal, point);
}

void Plane::Set(const Vector3& p0, const Vector3& p1, const Vector3& p2)
{
    Set(Math::Cross(p1 - p0, p2 - p0), p0);
}

void Plane::Set(const Vector3& normal, const Vector3& point)
{
    const Vector3 unit_normal = normal.Normalized();

    mData = Vector4{
    unit_normal.x,
    unit_normal.y,
    unit_normal.z,
    unit_normal.Dot(point),
    };
}

Vector3 Plane::GetNormal() const { return Vector3(mData.x, mData.y, mData.z); }

float Plane::GetDistance() const { return mData.w; }

DebugShape& Plane::DebugDraw(float size) const { return DebugDraw(size, size); }

DebugShape& Plane::DebugDraw(float sizeX, float sizeY) const
{
    return gDebugDrawer->DrawPlane(*this, sizeX, sizeY);
}

//-----------------------------------------------------------------------------Frustum
void Frustum::Set(const Vector3& lbn, const Vector3& rbn, const Vector3& rtn,
                  const Vector3& ltn, const Vector3& lbf, const Vector3& rbf,
                  const Vector3& rtf, const Vector3& ltf)
{
    mPoints[0] = lbn;
    mPoints[1] = rbn;
    mPoints[2] = rtn;
    mPoints[3] = ltn;
    mPoints[4] = lbf;
    mPoints[5] = rbf;
    mPoints[6] = rtf;
    mPoints[7] = ltf;

    // left
    mPlanes[0].Set(lbf, ltf, lbn);
    // right
    mPlanes[1].Set(rbn, rtf, rbf);
    // top
    mPlanes[2].Set(ltn, ltf, rtn);
    // bot
    mPlanes[3].Set(rbn, lbf, lbn);
    // near
    mPlanes[4].Set(lbn, ltn, rbn);
    // far
    mPlanes[5].Set(rbf, rtf, lbf);
}

Vector4* Frustum::GetPlanes() const { return (Vector4*)mPlanes; }

DebugShape& Frustum::DebugDraw() const
{
    return gDebugDrawer->DrawFrustum(*this);
}
