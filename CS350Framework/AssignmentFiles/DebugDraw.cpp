///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

#include "DebugDraw.hpp"

#include <Math/Math.hpp>
#include <Math/Reals.hpp>
#include <array>
#include <cstddef>
#include <vector>

#define ShowDebugDrawWarnings true

DebugDrawer* gDebugDrawer = new DebugDrawer();

//-----------------------------------------------------------------------------DebugShape
DebugShape::DebugShape()
{
    mColor = Vector4(.6f);
    mMask = (unsigned int)-1;
    mTimer = 0;
    mOnTop = false;
    mTransform.SetIdentity();
}

DebugShape& DebugShape::Color(const Vector4& color)
{
    mColor = color;
    return *this;
}

DebugShape& DebugShape::OnTop(bool state)
{
    mOnTop = state;
    return *this;
}

DebugShape& DebugShape::Time(float time)
{
    mTimer = time;
    return *this;
}

DebugShape& DebugShape::SetMaskBit(int bitIndex)
{
    mMask = 1 << bitIndex;
    return *this;
}

DebugShape& DebugShape::SetTransform(const Matrix4& transform)
{
    mTransform = transform;
    return *this;
}

//-----------------------------------------------------------------------------DebugDrawer
DebugDrawer::DebugDrawer()
{
    mActiveMask = (unsigned int)-1;
    mApplication = NULL;
}

void DebugDrawer::Update(float dt)
{
    std::vector<DebugShape> newShapes;
    for (size_t i = 0; i < mShapes.size(); ++i)
    {
        DebugShape& shape = mShapes[i];
        shape.mTimer -= dt;

        // If the shape still has time left then add it to the list of shapes to
        // keep drawing, anything that has a timer that ran out will not be in
        // the new list
        if (shape.mTimer >= 0)
            newShapes.push_back(shape);
    }

    mShapes.swap(newShapes);
}

void DebugDrawer::Draw()
{
    for (size_t i = 0; i < mShapes.size(); ++i)
    {
        DebugShape& shape = mShapes[i];

        // If the shape doesn't have one of the active mask bits set then don't
        // draw it
        if ((shape.mMask & mActiveMask) == 0)
            continue;

        // If this shape always draws on top then disable depth testing
        if (shape.mOnTop)
            glDisable(GL_DEPTH_TEST);


        // Decompose the matrix to set the gl transform (too lazy to properly
        // transform the matrix between formats)
        float radians;
        Vector3 scale, translation, axis;
        Matrix3 rotationMat;
        shape.mTransform.Decompose(&scale, &rotationMat, &translation);
        Math::ToAxisAngle(Math::ToQuaternion(rotationMat), &axis, &radians);
        glPushMatrix();
        // Set the transform
        glTranslatef(translation.x, translation.y, translation.z);
        glRotatef(Math::RadToDeg(radians), axis.x, axis.y, axis.z);
        glScalef(scale.x, scale.y, scale.z);

        glBegin(GL_LINES);
        glColor3fv(shape.mColor.array);

        // Draw all of the line segments of this shape
        for (size_t j = 0; j < shape.mSegments.size(); ++j)
        {
            LineSegment& segment = shape.mSegments[j];

            glVertex3fv(segment.mStart.array);
            glVertex3fv(segment.mEnd.array);
        }

        glEnd();
        glPopMatrix();

        // Make sure to re-enable depth testing
        if (shape.mOnTop)
            glEnable(GL_DEPTH_TEST);
    }
}

DebugShape& DebugDrawer::GetNewShape()
{
    mShapes.push_back(DebugShape());
    return mShapes.back();
}

DebugShape& DebugDrawer::DrawPoint(const Vector3& point)
{
    return DrawSphere(Sphere(point, 0.1f));
}

DebugShape& DebugDrawer::DrawLine(const LineSegment& line)
{
    // Draw a simple line
    DebugShape& shape = GetNewShape();
    shape.mSegments.push_back(line);

    return shape;
}

DebugShape& DebugDrawer::DrawRay(const Ray& ray, float t)
{
    // Draw a ray to a given t-length. The ray must have an arrow head for
    // visualization
    DebugShape& shape = GetNewShape();
    const Vector3 ray_end{ray.mStart + (t * ray.mDirection)};

    shape.mSegments.emplace_back(ray.mStart, ray_end);

    Vector3 w{1, 1, 1};
    w = w - w.Project(ray.mDirection);

    const Vector3 v{
    ray.mDirection.Cross(w),
    };

    const float r{0.1f * t};
    const size_t stacks{12};

    std::vector<Vector3> points{};

    for (size_t i{0}; i < stacks; i++)
    {
        const float theta{2.f * Math::cPi * (static_cast<float>(i) / stacks)};
        points.emplace_back(r * (v * Math::Cos(theta) + w * Math::Sin(theta)));
    }

    const float hat_length{0.1f * t};

    for (size_t i{0}; i < points.size(); i++)
    {
        const Vector3 curr_point{(ray_end + -hat_length * ray.mDirection) +
                                 points[i]};

        const Vector3 next_point{(ray_end + -hat_length * ray.mDirection) +
                                 points[(i + 1) % points.size()]};

        shape.mSegments.emplace_back(ray_end, curr_point);
        shape.mSegments.emplace_back(curr_point, next_point);
    }

    return shape;
}

DebugShape& DebugDrawer::DrawSphere(const Sphere& sphere)
{
    // Draw a sphere with 4 rings: x-axis, y-axis, z-axis, and the horizon disc.
    // Note: To access the camera's position for the horizon disc calculation
    // use mApplication->mCamera.mTranslation
    DebugShape& shape = GetNewShape();

    const auto DrawDisc{
    [](DebugShape& shape, const Vector3 center, const Vector3 dir,
       const float r)
    {
        Vector3 w{1, 1, 1};
        w = (w - w.Project(dir)).Normalized();

        const Vector3 v{
        dir.Cross(w).Normalized(),
        };

        std::vector<Vector3> points{};
        const size_t stacks{12};

        for (size_t i{0}; i < stacks; i++)
        {
            const float theta{2.f * Math::cPi *
                              (static_cast<float>(i) / stacks)};
            points.emplace_back(r *
                                (v * Math::Cos(theta) + w * Math::Sin(theta)));
        }

        for (size_t i{0}; i < points.size(); i++)
        {
            const Vector3 curr_point{center + points[i]};
            const Vector3 next_point{center + points[(i + 1) % points.size()]};

            shape.mSegments.emplace_back(curr_point, next_point);
        }
    },
    };

    DrawDisc(shape, sphere.mCenter, Vector3(1, 0, 0), sphere.mRadius);
    DrawDisc(shape, sphere.mCenter, Vector3(0, 1, 0), sphere.mRadius);
    DrawDisc(shape, sphere.mCenter, Vector3(0, 0, 1), sphere.mRadius);

    // TODO: Implement the horizon disc

    const Vector3 e{mApplication->mCamera.mTranslation};
    const float l{
    Math::Sqrt((e - sphere.mCenter).LengthSq() - Math::Sq(sphere.mRadius)),
    };

    const float horizon_radius{
    (sphere.mRadius * l) / (e - sphere.mCenter).Length(),
    };

    const float offset{
    Math::Sqrt(Math::Sq(sphere.mRadius) - Math::Sq(horizon_radius)),
    };

    const Vector3 center{
    sphere.mCenter - offset * (sphere.mCenter - e).Normalized(),
    };

    DrawDisc(shape, center, (sphere.mCenter - e).Normalized(), horizon_radius);

    return shape;
}

DebugShape& DebugDrawer::DrawAabb(const Aabb& aabb)
{
    // Draw all edges of an aabb. Make sure to not mis-match edges!
    DebugShape& shape = GetNewShape();
    const Vector3 extent{2.f * aabb.GetHalfSize()};

    const std::array<Vector3, 4> top{
    aabb.mMax,
    aabb.mMax - Vector3(extent.x, 0, 0),
    aabb.mMax - Vector3(extent.x, 0, extent.z),
    aabb.mMax - Vector3(0, 0, extent.z),
    };

    const std::array<Vector3, 4> bottom{
    aabb.mMin + Vector3(extent.x, 0, extent.z),
    aabb.mMin + Vector3(0, 0, extent.z),
    aabb.mMin,
    aabb.mMin + Vector3(extent.x, 0, 0),
    };

    for (size_t i{0}; i < top.size(); i++)
    {
        const size_t next_i{(i + 1) % top.size()};
        shape.mSegments.emplace_back(top[i], top[next_i]);
        shape.mSegments.emplace_back(bottom[i], bottom[next_i]);
        shape.mSegments.emplace_back(top[i], bottom[i]);
    }

    return shape;
}

DebugShape& DebugDrawer::DrawTriangle(const Triangle& triangle)
{
    // Draw the 3 edges of a triangles
    DebugShape& shape = GetNewShape();

    for (size_t i{0}; i < 3; i++)
    {
        shape.mSegments.emplace_back(triangle.mPoints[i],
                                     triangle.mPoints[(i + 1) % 3]);
    }

    return shape;
}

DebugShape& DebugDrawer::DrawPlane(const Plane& plane, float sizeX, float sizeY)
{
    // Draw a quad with a normal at the plane's center.
    const Vector3 normal{plane.mData.x, plane.mData.y, plane.mData.z};
    const Vector3 center{normal * plane.GetDistance()};

    DebugShape& shape{DrawRay({center, normal}, 5.f)};

    Vector3 w{1, 1, 1};
    w = (w - w.Project(normal)).Normalized();

    const Vector3 v{
    normal.Cross(w).Normalized(),
    };

    const float half_x{sizeX / 2.f};
    const float half_y{sizeY / 2.f};

    std::array<Vector3, 4> points{
    center + w * half_x + v * half_y,
    center - w * half_x + v * half_y,
    center - w * half_x - v * half_y,
    center + w * half_x - v * half_y,
    };

    for (size_t i{0}; i < points.size(); i++)
    {
        shape.mSegments.emplace_back(points[i],
                                     points[(i + 1) % points.size()]);
    }

    return shape;
}

DebugShape& DebugDrawer::DrawQuad(const Vector3& p0, const Vector3& p1,
                                  const Vector3& p2, const Vector3& p3)
{
    // Draw the4 edges of a quad. Make sure to look at this and make sure the
    // quad is not bow-tied.
    DebugShape& shape = GetNewShape();

    shape.mSegments.emplace_back(p0, p1);
    shape.mSegments.emplace_back(p1, p2);
    shape.mSegments.emplace_back(p2, p3);
    shape.mSegments.emplace_back(p3, p0);

    return shape;
}

DebugShape& DebugDrawer::DrawFrustum(const Frustum& frustum)
{
    // Draw the 6 faces of the frustum using the 8 frustum points.
    // See Frustum::Set for the point order. For example, Points[4] is
    // left-bottom-front.
    DebugShape& shape = GetNewShape();
    const Vector3* points{frustum.mPoints};

    for (size_t i{0}; i < 4; i++)
    {
        shape.mSegments.emplace_back(points[i], points[(i + 1) % 4]);
        shape.mSegments.emplace_back(points[i + 4], points[((i + 1) % 4) + 4]);
        shape.mSegments.emplace_back(points[i], points[i + 4]);
    }

    return shape;
}
