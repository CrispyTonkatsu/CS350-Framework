///////////////////////////////////////////////////////////////////////////////
///
///  \file EulerAngles.hpp
///  Declaration of the EulerAngle structure, interface referenced from
///  Insomniac Games, implementation referenced from Graphics Gems IV.
///
///  Authors: Benjamin Strukus
///  Copyright 2010-2012, DigiPen Institute of Technology
///
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include "EulerOrder.hpp"
#include "Vector3.hpp"

namespace Math
{
    /// Forward declaration
    struct Matrix3;
    using Mat3Param = const Matrix3&;
    using Mat3Ref = Matrix3&;

    /// Forward declaration
    struct Matrix4;
    using Mat4Param = const Matrix4&;
    using Mat4Ref = Matrix4&;

    /// Forward declaration
    struct Quaternion;
    using QuatParam = const Quaternion&;
    using QuatRef = Quaternion&;

    struct EulerAngles;
    using EulerAnglesParam = const EulerAngles&;
    using EulerAnglesRef = EulerAngles&;
    using EulerAnglesPtr = EulerAngles*;

    //----------------------------------------------------------------- Euler
    //Angles
    /// Structure to provide a consistent way to deal with rotations stored as
    /// Euler angles.
    struct EulerAngles
    {
        EulerAngles(EulerOrderParam order);
        EulerAngles(Vec3Param xyzRotation, EulerOrderParam order);
        EulerAngles(float xRotation, float yRotation, float zRotation,
                    EulerOrderParam order);
        EulerAngles(Mat3Param matrix, EulerOrderParam order);
        EulerAngles(Mat4Param matrix, EulerOrderParam order);
        EulerAngles(QuatParam quaternion, EulerOrderParam order);

        /// Index operator to access the Angles data directly.
        float operator[](unsigned index) const;

        /// Index operator to access the Angles data directly.
        float& operator[](unsigned index);


        float I() const;
        float J() const;
        float K() const;
        float H() const;

        void I(float i);
        void J(float j);
        void K(float k);
        void H(float h);

        void Reorder(EulerOrderParam newOrder);

        Vector3 Angles;
        EulerOrder Order;
    };
} // namespace Math
