using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DecodedDescription 
{
    public int FrameNr;
    public int DescriptionNr;
    public int NPoints;
    public byte[] Data;

    public DecodedDescription(int FrameNr, int DescriptionNr, int NPoints, byte[] Data) {
        this.FrameNr = FrameNr;
        this.DescriptionNr = DescriptionNr;
        this.NPoints = NPoints;
        this.Data = Data;
    }
}
