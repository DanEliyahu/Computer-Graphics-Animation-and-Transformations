using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

public class CharacterAnimator : MonoBehaviour
{
    public TextAsset BVHFile; // The BVH file that defines the animation and skeleton
    public bool animate; // Indicates whether or not the animation should be running

    private BVHData data; // BVH data of the BVHFile will be loaded here
    private int currFrame = 0; // Current frame of the animation
    private float waiting;
    private const int scaleFactor = 2;

    // Start is called before the first frame update
    void Start()
    {
        BVHParser parser = new BVHParser();
        data = parser.Parse(BVHFile);
        waiting = data.frameLength;
        CreateJoint(data.rootJoint, Vector3.zero);
    }

    // Returns a Matrix4x4 representing a rotation aligning the up direction of an object with the given v
    Matrix4x4 RotateTowardsVector(Vector3 v)
    {
        // applying the same method like in the TA this time we only need to do the inverse rotation 
        Vector3 normalize = v.normalized;
        float thetaX = -(Mathf.PI / 2 - Mathf.Atan2(normalize.y, normalize.z));
        Matrix4x4 rX = MatrixUtils.RotateX(Mathf.Rad2Deg * thetaX);
        Matrix4x4 rXInverse = rX.inverse;
        float thetaZ = (Mathf.PI / 2 -
                        Mathf.Atan2(Mathf.Sqrt(normalize.z * normalize.z + normalize.y * normalize.y), normalize.x));
        Matrix4x4 rZ = MatrixUtils.RotateZ(Mathf.Rad2Deg * thetaZ);
        Matrix4x4 rZInverse = rZ.inverse;
        // Your code here
        return rXInverse * rZInverse;
    }

    // Creates a Cylinder GameObject between two given points in 3D space
    GameObject CreateCylinderBetweenPoints(Vector3 p1, Vector3 p2, float diameter)
    {
        GameObject cyl = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        Matrix4x4 T = MatrixUtils.Translate((p1 + p2) / 2);
        Matrix4x4 R = RotateTowardsVector(p2 - p1);
        Vector3 distance = p2 - p1;
        Matrix4x4 S = MatrixUtils.Scale(new Vector3(diameter, (0.5f) * (distance.magnitude), diameter));
        Matrix4x4 action = T * R * S;
        MatrixUtils.ApplyTransform(cyl, action);
        return cyl;
    }

    // Creates a GameObject representing a given BVHJoint and recursively creates GameObjects for it's child joints
    GameObject CreateJoint(BVHJoint joint, Vector3 parentPosition)
    {
        joint.gameObject = new GameObject(joint.name);
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        sphere.transform.parent = joint.gameObject.transform;
        if (String.Equals(joint.name, "Head"))
        {
            MatrixUtils.ApplyTransform(sphere,
                MatrixUtils.Scale(new Vector3(4 * scaleFactor, 4 * scaleFactor, 4 * scaleFactor)));
        }
        else
        {
            MatrixUtils.ApplyTransform(sphere, MatrixUtils.Scale(new Vector3(scaleFactor, scaleFactor, scaleFactor)));
        }

        MatrixUtils.ApplyTransform(joint.gameObject, MatrixUtils.Translate(parentPosition + joint.offset));
        if (joint.isEndSite)
        {
            return joint.gameObject;
        }

        foreach (BVHJoint child in joint.children)
        {
            Vector3 pos = joint.gameObject.transform.position;
            CreateJoint(child, pos);
            GameObject bone = CreateCylinderBetweenPoints(pos, child.gameObject.transform.position, 0.5f);
            bone.transform.parent = joint.gameObject.transform;
        }

        return joint.gameObject;
    }


    // Transforms BVHJoint according to the keyframe channel data, and recursively transforms its children
    private void TransformJoint(BVHJoint joint, Matrix4x4 parentTransform, float[] keyframe)
    {
        Matrix4x4 t = MatrixUtils.Translate(joint.offset);
        if (joint.isEndSite) // EndSite has no Channels
        {
            MatrixUtils.ApplyTransform(joint.gameObject, parentTransform * t);
            return;
        }

        Vector3Int index = joint.rotationChannels;
        Vector3Int order = joint.rotationOrder;
        Matrix4x4 rx = MatrixUtils.RotateX(keyframe[index.x]);
        Matrix4x4 ry = MatrixUtils.RotateY(keyframe[index.y]);
        Matrix4x4 rz = MatrixUtils.RotateZ(keyframe[index.z]);
        Matrix4x4[] rotateArray = new Matrix4x4[3]; // create array to hold order of rotations
        rotateArray[order.x] = rx;
        rotateArray[order.y] = ry;
        rotateArray[order.z] = rz;
        Matrix4x4 rotations = rotateArray[0] * rotateArray[1] * rotateArray[2];
        Matrix4x4 m = t * rotations;
        Matrix4x4 action = parentTransform * m;
        MatrixUtils.ApplyTransform(joint.gameObject, action);

        foreach (BVHJoint child in joint.children)
        {
            TransformJoint(child, action, keyframe);
        }
    }

    // Update is called once per frame
    void Update()
    {
        if (animate)
        {
            waiting -= Time.deltaTime;
            if (waiting <= 0)
            {
                float[] currentKeyFrame = data.keyframes[currFrame];
                Vector3Int translateChannel = data.rootJoint.positionChannels;
                Vector3 newPos = new Vector3(currentKeyFrame[translateChannel.x], currentKeyFrame[translateChannel.y],
                    currentKeyFrame[translateChannel.z]);
                Matrix4x4 trans = MatrixUtils.Translate(newPos);
                TransformJoint(data.rootJoint, trans, currentKeyFrame);
                currFrame++;
                currFrame %= data.numFrames;
                waiting = data.frameLength;
            }
        }
    }
}