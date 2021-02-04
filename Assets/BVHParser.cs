using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;


public class BVHData
{
    public BVHJoint rootJoint; // Root BVHJoint object
    public int numFrames; // Number of frames in the animation 
    public float frameLength; // Length of each frame in seconds
    public List<float[]> keyframes; // Keyframes containing channel data for animating

    public BVHData()
    {
        keyframes = new List<float[]>();
    }
}


public class BVHJoint
{
    public string name; // Name of the joint
    public Vector3 offset; // Offset relative to the parent position
    public Vector3Int rotationChannels; // XYZ rotation channel indices in keyframe data
    public Vector3Int rotationOrder; // Order in which XYZ rotations need to be performed
    public Vector3Int positionChannels; // XYZ position channel indices in keyframe data (Optional!)
    public List<BVHJoint> children; // List of children BVHJoints
    public GameObject gameObject; // Will hold GameObject of this joint in the scene
    public bool isEndSite; // Indicates if this joint an EndSite or not

    public BVHJoint(string _name, bool _isEndSite)
    {
        name = _name;
        children = new List<BVHJoint>();
        isEndSite = _isEndSite;
    }
}



public class BVHParser
{
    private BVHData parsedData;

    private static readonly int KEYWORD = 0;
    private static readonly int NAME = 1;
    private static readonly int NUM_CHANNELS = 1;
    private static readonly string[] AXES = { "X", "Y", "Z" };
    private static readonly int X = 1;
    private static readonly int Y = 2;
    private static readonly int Z = 3;

    private string[] lines;
    private int currLine;
    private int currChannelIndex;

    public BVHData Parse(TextAsset BVHFile)
    {
        lines = BVHFile.text.Split('\n');
        currLine = 0;
        currChannelIndex = 0;
        parsedData = new BVHData();

        string[] line;
        while ((line = getNextLine()) != null)
        {
            if (line[KEYWORD] == "HIERARCHY")
            {
                ParseHierarchySection();
            }
            else if (line[KEYWORD] == "MOTION")
            {
                ParseMotionSection();
            }
            else
            {
                PrintError("Unkown command in BVH data");
            }
        }
        Debug.Log("Finished Parsing BVH");
        return parsedData;
    }

    private string[] getNextLine()
    {
        if (currLine < lines.Length)
        {
            // Trim any leading or trailing whitespace, then split into tokens also by whitespace
            var line = lines[currLine].Trim();
            var tokens = line.Split(new char[0], StringSplitOptions.RemoveEmptyEntries);
            currLine++;
            return tokens.Length > 0 ? tokens : getNextLine();
        }
        return null;
    }

    private void ParseHierarchySection()
    {
        var line = getNextLine();
        if (line[KEYWORD] == "ROOT")
        {
            parsedData.rootJoint = ParseJoint(line[NAME]);
        }
        else
        {
            PrintError("No ROOT joint found in BVH file");
        }
    }

    private BVHJoint ParseJoint(string name, bool isEnd = false)
    {
        BVHJoint joint = new BVHJoint(name, isEnd);
        var line = getNextLine();
        while (line[KEYWORD] != "}")
        {
            if (line[KEYWORD] == "OFFSET")
            {
                joint.offset = new Vector3(float.Parse(line[X], CultureInfo.InvariantCulture),
                                             float.Parse(line[Y], CultureInfo.InvariantCulture),
                                             float.Parse(line[Z], CultureInfo.InvariantCulture));
            }
            else if (line[KEYWORD] == "CHANNELS")
            {
                var numChannels = int.Parse(line[NUM_CHANNELS]);
                int rotationIndex = 0;

                for (var c = 0; c < numChannels; c++)
                {
                    var channelName = line[c + 2];

                    var axis = Array.IndexOf(AXES, channelName.Substring(0, 1));
                    if (channelName.Substring(1) == "rotation")
                    {
                        joint.rotationChannels[axis] = currChannelIndex;
                        joint.rotationOrder[axis] = rotationIndex;
                        rotationIndex++;
                    }
                    else if (channelName.Substring(1) == "position")
                    {
                        joint.positionChannels[axis] = currChannelIndex;
                    }

                    currChannelIndex++;
                }
            }
            else if (line[KEYWORD] == "JOINT")
            {
                joint.children.Add(ParseJoint(line[NAME]));
            }
            else if (line[KEYWORD] == "End")
            {
                joint.children.Add(ParseJoint("End", true));
            }
            line = getNextLine();
        }
        return joint;
    }

    private void ParseMotionSection()
    {
        string[] line;
        while ((line = getNextLine()) != null)
        {
            if (line[KEYWORD] == "Frames:")
            {
                parsedData.numFrames = int.Parse(line[1]);
            }
            else if (line[KEYWORD] == "Frame")
            {
                parsedData.frameLength = float.Parse(line[2], CultureInfo.InvariantCulture);
            }
            else
            {
                var keyframe = new float[currChannelIndex];
                if (line.Length != currChannelIndex)
                {
                    PrintError("Wrong number of channels in keyframe");
                }
                for (var i = 0; i < currChannelIndex; i++)
                {
                    keyframe[i] = float.Parse(line[i], CultureInfo.InvariantCulture);
                }
                parsedData.keyframes.Add(keyframe);
            }
        }
    }

    private void PrintError(string message)
    {
        Debug.LogError("BVH line " + currLine + ": " + message);
    }
}
