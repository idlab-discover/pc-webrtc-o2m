using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using UnityEngine;

public class DecodedPointCloud
{
    public int FrameNr;
    public int NPoints;
    public int MaxDescriptions;
    public int CurrentNDescriptions;
    public int Quality = 0;
    public List<Vector3> Points;
    public List<Color32> Colors;
    public List<DecodedDescription> Descriptions;
    public bool IsCompleted { get { return MaxDescriptions == CurrentNDescriptions; } } 
    private Mutex mut = new Mutex();
    public DecodedPointCloud(int FrameNr, int MaxDescriptions)
    {
        this.FrameNr = FrameNr;
        this.MaxDescriptions = MaxDescriptions;
        Descriptions = new List<DecodedDescription>(this.MaxDescriptions);
        Points = new List<Vector3>();
        Colors = new List<Color32>();
    }
    public void LockClass()
    {
        mut.WaitOne();
    }
    public void UnlockClass()
    {
        mut.ReleaseMutex();
    }
}