
using System;
using UnityEngine;

public static class MatrixUtils
{
    private static readonly float EPSILON = 0.001f;

    // Applies the transformation matrix m to given gameObject’s transform component.
    // Note that this overwrites any previous transformations made to gameObject.
    public static void ApplyTransform(GameObject gameObject, Matrix4x4 m)
    {
        Transform transform = gameObject.transform;

        transform.position = m.GetColumn(3);
        transform.localRotation = Quaternion.LookRotation(
            m.GetColumn(2),
            m.GetColumn(1)
        );
        transform.localScale = new Vector3(m.GetColumn(0).magnitude,
                                           m.GetColumn(1).magnitude,
                                           m.GetColumn(2).magnitude);
    }

    // Returns a Matrix4x4 representing a rotation of angleDeg degrees around the Z axis
    public static bool CheckEquality(Matrix4x4 m1, Matrix4x4 m2)
    {
        var areEqual = true;
        for (var i = 0; i < 4; i++)
        {
            areEqual = areEqual && (EPSILON > (m1.GetRow(i) - m2.GetRow(i)).magnitude);
        }
        return areEqual;
    }

    // Returns a Matrix4x4 representing a translation to the given position
    public static Matrix4x4 Translate(Vector3 position)
    {
        var m = Matrix4x4.identity; // 1   0   0   x
        m.m03 = position.x;        // 0   1   0   y
        m.m13 = position.y;        // 0   0   1   z
        m.m23 = position.z;        // 0   0   0   1
        return m;
    }

    // Returns a Matrix4x4 representing a scale on each axis according to the given scale
    public static Matrix4x4 Scale(Vector3 scale)
    {
        var m = Matrix4x4.identity; //  sx   0   0   0
        m.m00 = scale.x;           //   0  sy   0   0
        m.m11 = scale.y;           //   0   0  sz   0
        m.m22 = scale.z;           //   0   0   0   1
        return m;
    }

    // Returns a Matrix4x4 representing a rotation of angleDeg degrees around the X axis
    public static Matrix4x4 RotateX(float angleDeg)
    {
        float aAngleRad = angleDeg * Mathf.Deg2Rad;
        Matrix4x4 m = Matrix4x4.identity;     //  1   0   0   0 
        m.m11 = m.m22 = Mathf.Cos(aAngleRad); //  0  cos -sin 0
        m.m21 = Mathf.Sin(aAngleRad);         //  0  sin  cos 0
        m.m12 = -m.m21;                       //  0   0   0   1
        return m;
    }

    // Returns a Matrix4x4 representing a rotation of angleDeg degrees around the Y axis
    public static Matrix4x4 RotateY(float angleDeg)
    {
        float aAngleRad = angleDeg * Mathf.Deg2Rad;
        Matrix4x4 m = Matrix4x4.identity;     // cos  0  sin  0
        m.m00 = m.m22 = Mathf.Cos(aAngleRad); //  0   1   0   0
        m.m02 = Mathf.Sin(aAngleRad);         //-sin  0  cos  0
        m.m20 = -m.m02;                       //  0   0   0   1
        return m;
    }

    // Returns a Matrix4x4 representing a rotation of angleDeg degrees around the Z axis
    public static Matrix4x4 RotateZ(float angleDeg)
    {
        float aAngleRad = angleDeg * Mathf.Deg2Rad;
        Matrix4x4 m = Matrix4x4.identity;     // cos -sin 0   0
        m.m00 = m.m11 = Mathf.Cos(aAngleRad); // sin  cos 0   0
        m.m10 = Mathf.Sin(aAngleRad);         //  0   0   0   0
        m.m01 = -m.m10;                       //  0   0   0   1
        return m;
    }

    // Returns a Matrix4x4 representing a rotation aligning the up vector (0, 1, 0) of an
    // object with the given direction vector
    public static Matrix4x4 RotateTowardsVector(Vector3 v)
    {
        Quaternion rotation = Quaternion.FromToRotation(Vector3.up, v);
        return Matrix4x4.Rotate(rotation);
    }
}