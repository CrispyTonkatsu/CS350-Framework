///////////////////////////////////////////////////////////////////////////////
///
/// Authors: Joshua Davis
/// Copyright 2015, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#include "Precompiled.hpp"

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
  for(size_t i = 0; i < mShapes.size(); ++i)
  {
    DebugShape& shape = mShapes[i];
    shape.mTimer -= dt;

    // If the shape still has time left then add it to the list of shapes to keep drawing,
    // anything that has a timer that ran out will not be in the new list
    if(shape.mTimer >= 0)
      newShapes.push_back(shape);
  }

  mShapes.swap(newShapes);
}

void DebugDrawer::Draw()
{
  for(size_t i = 0; i < mShapes.size(); ++i)
  {
    DebugShape& shape = mShapes[i];

    // If the shape doesn't have one of the active mask bits set then don't draw it
    if((shape.mMask & mActiveMask) == 0)
      continue;
    
    // If this shape always draws on top then disable depth testing
    if(shape.mOnTop)
      glDisable(GL_DEPTH_TEST);


    // Decompose the matrix to set the gl transform (too lazy to properly transform the matrix between formats)
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
    for(size_t j = 0; j < shape.mSegments.size(); ++j)
    {
      LineSegment& segment = shape.mSegments[j];

      glVertex3fv(segment.mStart.array);
      glVertex3fv(segment.mEnd.array);
    }

    glEnd();
    glPopMatrix();

    // Make sure to re-enable depth testing
    if(shape.mOnTop)
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
  /******Student:Assignment2******/
  // Draw a simple line
  DebugShape& shape = GetNewShape();
  WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
  return shape;
}

DebugShape& DebugDrawer::DrawRay(const Ray& ray, float t)
{
  /******Student:Assignment2******/
  // Draw a ray to a given t-length. The ray must have an arrow head for visualization
  DebugShape& shape = GetNewShape();
  WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
  return shape;
}

DebugShape& DebugDrawer::DrawSphere(const Sphere& sphere)
{
  /******Student:Assignment2******/
  // Draw a sphere with 4 rings: x-axis, y-axis, z-axis, and the horizon disc.
  // Note: To access the camera's position for the horizon disc calculation use mApplication->mCamera.mTranslation
  DebugShape& shape = GetNewShape();
  WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
  return shape;
}

DebugShape& DebugDrawer::DrawAabb(const Aabb& aabb)
{
  /******Student:Assignment2******/
  // Draw all edges of an aabb. Make sure to not mis-match edges!
  DebugShape& shape = GetNewShape();  
  WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
  return shape;
}

DebugShape& DebugDrawer::DrawTriangle(const Triangle& triangle)
{
  /******Student:Assignment2******/
  // Draw the 3 edges of a triangles
  DebugShape& shape = GetNewShape();
  WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
  return shape;
}

DebugShape& DebugDrawer::DrawPlane(const Plane& plane, float sizeX, float sizeY)
{
  /******Student:Assignment2******/
  // Draw a quad with a normal at the plane's center.
  DebugShape& shape = GetNewShape();
  WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
  return shape;
}

DebugShape& DebugDrawer::DrawQuad(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
  /******Student:Assignment2******/
  // Draw the4 edges of a quad. Make sure to look at this and make sure the quad is not bow-tied.
  DebugShape& shape = GetNewShape();
  WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
  return shape;
}

DebugShape& DebugDrawer::DrawFrustum(const Frustum& frustum)
{
  /******Student:Assignment2******/
  // Draw the 6 faces of the frustum using the 8 frustum points.
  // See Frustum.Set for the point order. For example, Points[4] is left-bottom-front.
  DebugShape& shape = GetNewShape();
  WarnIf(ShowDebugDrawWarnings, "Assignment2: Required function un-implemented");
  return shape;
}
