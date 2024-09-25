package main

type FramePacket struct {
	FrameNr   uint32
	FrameLen  uint32
	SeqOffset uint32
	SeqLen    uint32
	Data      [1180]byte
}

func NewFramePacket(frameNr, frameLen, seqLen, seqOffset uint32, dataSubArray []byte) *FramePacket {
	packet := &FramePacket{
		FrameNr:   frameNr,
		FrameLen:  frameLen,
		SeqOffset: seqOffset,
		SeqLen:    seqLen,
	}
	copy(packet.Data[:], dataSubArray[seqOffset:(seqOffset+seqLen)])
	return packet
}
