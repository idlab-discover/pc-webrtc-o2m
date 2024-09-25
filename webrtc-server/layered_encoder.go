package main

import (
	"bytes"
	"encoding/binary"
	"math"
	"unsafe"

	"github.com/Workiva/go-datastructures/queue"
)

type MultiLayerMainHeader struct {
	NLayers uint32
	MinX    float32
	MinY    float32
	MinZ    float32
	MaxX    float32
	MaxY    float32
	MaxZ    float32
}

type MultiLayerSideHeader struct {
	LayerID  uint32
	FrameLen uint32
}

type FrameCategory struct {
	Dst            float32
	FrameID        uint32
	CurrentBitrate uint32
	CurrentCat     uint8
	Combo          uint8
}

type LayeredEncoder struct {
	Bitrate uint32
}

type DstAndFrameID struct {
	Dst     float32
	FrameID uint32
}

/*type PanZoom struct {
	XPos  float32
	YPos  float32
	ZPos  float32
	Yaw   float32
	Pitch float32
	Roll  float32
}*/

func NewLayeredEncoder() *LayeredEncoder {
	return &LayeredEncoder{}
}
func (lhs FrameCategory) Compare(other queue.Item) int {
	rhs := other.(FrameCategory)
	if lhs.Combo == rhs.Combo && lhs.Dst == rhs.Dst {
		return 0
	}
	if lhs.Combo < rhs.Combo || lhs.Dst < rhs.Dst {
		return -1
	}
	return 1
}

func (l *LayeredEncoder) EncodeMultiFrame(frame []byte, bitrate uint32) []byte {
	//
	var offsets []uint32
	//var distanceToUser []float32
	var distanceIDs []uint8
	var mainLHeader MultiLayerMainHeader
	var distanceToCategory []*queue.PriorityQueue
	for i := 0; i < 3; i++ {
		q := queue.NewPriorityQueue(10, false)
		distanceToCategory = append(distanceToCategory, q)
	}
	pz := PanZoom{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
	// Combos
	cs := [][][]uint8{
		{
			{0},
			{0, 2},
			{0, 1},
			{0, 1, 2},
		},
		{
			{1},
			{1, 2},
		},
		{
			{2},
		},
	}
	//categorySizesForFrames := [][]uint32{}
	totalBandwidthForCategoryForCombos := [][]uint32{
		{0, 0, 0, 0},
		{0, 0},
		{0},
	}

	buf := bytes.NewBuffer(frame[:unsafe.Sizeof(mainLHeader)])
	if err := binary.Read(buf, binary.LittleEndian, &mainLHeader); err != nil {
		panic(err)
	}
	offsets = append(offsets, uint32(unsafe.Sizeof(mainLHeader)))

	mx := mainLHeader.MinX + (mainLHeader.MaxX-mainLHeader.MinX)/2
	my := mainLHeader.MinY + (mainLHeader.MaxY-mainLHeader.MinY)/2
	mz := mainLHeader.MinZ + (mainLHeader.MaxZ-mainLHeader.MinZ)/2
	dst := math.Sqrt(math.Pow(float64(pz.XPos-mx), 2) + math.Pow(float64(pz.YPos-my), 2) + math.Pow(float64(pz.ZPos-mz), 2))
	dstID := uint8(dst / 50)
	distanceIDs = append(distanceIDs, dstID)
	// Frame is out of range
	if dstID > 3 {
		return nil
	}

	sHeaders := make([]MultiLayerSideHeader, mainLHeader.NLayers)
	lOffsets := make([]uint32, mainLHeader.NLayers)
	lLen := make([]uint32, mainLHeader.NLayers)

	currentOffset := uint32(unsafe.Sizeof(mainLHeader))

	for j := 0; j < int(mainLHeader.NLayers); j++ {
		var shTemp MultiLayerSideHeader
		buf := bytes.NewBuffer(frame[currentOffset:(currentOffset + uint32(unsafe.Sizeof(mainLHeader)))])
		if err := binary.Read(buf, binary.LittleEndian, &shTemp); err != nil {
			panic(err)
		}
		sHeaders = append(sHeaders, shTemp)
		lOffsets[shTemp.LayerID] = uint32(currentOffset)
		currentOffset += uint32(unsafe.Sizeof(shTemp)) + shTemp.FrameLen
		lLen[shTemp.LayerID] = uint32(unsafe.Sizeof(shTemp)) + shTemp.FrameLen
	}
	for j := 0; j < len(cs); j++ {
		for k := 0; k < len(cs[j]); k++ {
			lc := cs[j][k]
			lcSize := uint32(0)
			for _, l := range lc {
				lcSize += lLen[l]
			}
			totalBandwidthForCategoryForCombos[j][k] += lcSize
		}
	}

	tempBitrate := int(bitrate / 8 / 30)
	// Check if current category
	foundCat := -1
	foundCombo := -1
	for i := int(dstID); i < 3 && foundCat < 0; i++ {
		// Loop over category
		for j := len(cs[i]) - 1; j >= 0 && foundCat < 0; j-- {
			// Loop over category
			if totalBandwidthForCategoryForCombos[i][j]+uint32(unsafe.Sizeof(MultiLayerMainHeader{})) <= uint32(tempBitrate) {
				// Found good combo
				foundCat = i
				foundCombo = j
			}
		}
	}

	// Not enough bitrate for any version
	if foundCat == -1 {
		return nil
	}

	size := uint32(unsafe.Sizeof(MultiLayerMainHeader{})) + totalBandwidthForCategoryForCombos[foundCat][foundCombo]
	mainLHeader.NLayers = uint32(len(cs[foundCat][foundCombo]))
	multiFrame := make([]byte, size)
	buf = new(bytes.Buffer)
	if err := binary.Write(buf, binary.LittleEndian, mainLHeader); err != nil {
		panic(err)
	}
	copy(multiFrame[:], buf.Bytes()[:])
	offset := uint32(len(buf.Bytes()))
	for i := 0; i < len(lLen); i++ {
		for j := 0; j < len(cs[foundCat][foundCombo]); j++ {
			if uint8(i) == cs[foundCat][foundCombo][j] {
				copy(multiFrame[offset:], frame[lOffsets[i]:lOffsets[i]+lLen[i]])
				offset += lLen[i]
			}
		}
	}

	return multiFrame
}
