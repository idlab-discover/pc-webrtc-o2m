using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DecodeTask
{
    public int FrameNr;
    public int DescriptionNr;
    public int NDescriptions;
    public int DescriptionSize;
    public int Quality;
    public byte[] DescriptionData;
    public DecodeTask(int FrameNr, int DescriptionNr, int NDescriptions, int DescriptionSize, int Quality, byte[] DescriptionData) {
        this.FrameNr = FrameNr; 
        this.DescriptionNr = DescriptionNr;
        this.NDescriptions = NDescriptions;
        this.DescriptionSize = DescriptionSize;
        this.Quality = Quality;
        this.DescriptionData = DescriptionData;
    }
}
