package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"io/ioutil"
	"os"
	"path/filepath"
	"time"
)

type Frame struct {
	ClientID uint32
	FrameLen uint32
	FrameNr  uint32
	Data     []byte
}

type Transcoder interface {
	UpdateBitrate(bitrate uint32)
	UpdateProjection()
	EncodeFrame(data []byte, framecounter uint32, bitrate uint32) *Frame
	IsReady() bool
	GetEstimatedBitrate() uint32
	GetFrameCounter() uint32
	NextFrame() (uint32, []byte)
}

func readFiles(directory string) ([][]byte, []int64, error) {
	var fileContents [][]byte
	var fileSizes []int64

	files, err := ioutil.ReadDir(directory)
	if err != nil {
		return nil, nil, err
	}

	for _, file := range files {
		filePath := filepath.Join(directory, file.Name())

		// Read file content
		content, err := os.ReadFile(filePath)
		if err != nil {
			return nil, nil, err
		}
		fileContents = append(fileContents, content)

		// Get file size
		fileSizes = append(fileSizes, file.Size())
	}

	return fileContents, fileSizes, nil
}

type TranscoderFiles struct {
	frameCounter     uint32
	isReady          bool
	fileCounter      uint32
	lEnc             *LayeredEncoder
	estimatedBitrate uint32
	prevFrameTime    int64
	frameRate        uint32

	frames [][]byte
}

func (f *Frame) Bytes() []byte {
	buf := new(bytes.Buffer)
	binary.Write(buf, binary.BigEndian, f.ClientID)
	binary.Write(buf, binary.BigEndian, f.FrameLen)
	binary.Write(buf, binary.BigEndian, f.FrameNr)
	binary.Write(buf, binary.BigEndian, f.Data)
	return buf.Bytes()
}

func NewTranscoderFile(contentDirectory string, frameRate uint32) *TranscoderFiles {
	//fBytes, _ := ReadBinaryFiles(contentDirectory)
	frames, _, err := readFiles(contentDirectory)
	if err != nil {
		fmt.Println("Error reading layer_0:", err)
	}

	return &TranscoderFiles{0, true, 0, NewLayeredEncoder(), 0, 0, frameRate, frames}
}

func (t *TranscoderFiles) UpdateBitrate(bitrate uint32) {
	t.estimatedBitrate = uint32(float64(bitrate) * 0.9)
	t.lEnc.Bitrate = uint32(float64(bitrate) * 0.9)
}

func (t *TranscoderFiles) UpdateProjection() {
	// Do nothing
}

func (t *TranscoderFiles) NextFrame() (uint32, []byte) {
	sleepTime := int64(1000/t.frameRate) - (time.Now().UnixMilli() - t.prevFrameTime)
	if sleepTime > 0 {
		time.Sleep(time.Duration(sleepTime) * time.Millisecond)
	}
	t.prevFrameTime = time.Now().UnixMilli()
	t.frameCounter++
	currentCounter := t.fileCounter
	t.fileCounter = (t.fileCounter + 1) % uint32(len(t.frames))
	return t.frameCounter, t.frames[currentCounter]
}

func (t *TranscoderFiles) EncodeFrame(data []byte, framecounter uint32, bitrate uint32) *Frame {

	//transcodedData := t.lEnc.EncodeMultiFrame(data)

	transcodedData := t.lEnc.EncodeMultiFrame(data, bitrate)
	if data == nil {
		return nil
	}
	rFrame := Frame{0, uint32(len(transcodedData)), framecounter, transcodedData}
	return &rFrame
}

func (t *TranscoderFiles) IsReady() bool {
	return t.isReady
}

func (t *TranscoderFiles) GetEstimatedBitrate() uint32 {
	return t.estimatedBitrate
}

func (t *TranscoderFiles) GetFrameCounter() uint32 {
	return t.frameCounter
}

type TranscoderRemote struct {
	proxyConn        *ProxyConnection
	frameCounter     uint32
	isReady          bool
	lEnc             *LayeredEncoder
	estimatedBitrate uint32
}

func NewTranscoderRemote(proxy_con *ProxyConnection) *TranscoderRemote {
	return &TranscoderRemote{proxy_con, 0, true, NewLayeredEncoder(), 0}
}

func (t *TranscoderRemote) UpdateBitrate(bitrate uint32) {
	t.estimatedBitrate = uint32(float64(bitrate) * 0.9)
	t.lEnc.Bitrate = uint32(float64(bitrate) * 0.9)
}

func (t *TranscoderRemote) UpdateProjection() {
	// Do nothing
}
func (t *TranscoderRemote) NextFrame() (uint32, []byte) {
	return proxyConn.NextFrame(0)
}

func (t *TranscoderRemote) EncodeFrame(data []byte, framecounter uint32, bitrate uint32) *Frame {
	transcodedData := t.lEnc.EncodeMultiFrame(data, bitrate)
	if data == nil {
		return nil
	}
	rFrame := Frame{0, uint32(len(transcodedData)), framecounter, transcodedData}
	return &rFrame
}

func (t *TranscoderRemote) IsReady() bool {
	return t.isReady
}
func (t *TranscoderRemote) GetEstimatedBitrate() uint32 {
	return t.estimatedBitrate
}
func (t *TranscoderRemote) GetFrameCounter() uint32 {
	return t.frameCounter
}

// INDI TRANSCODER

type TranscoderRemoteIndi struct {
	proxyConn        *ProxyConnection
	frameCounter     uint32
	isReady          bool
	estimatedBitrate uint32
	clientID         uint32
}

func NewTranscoderRemoteIndi(proxy_con *ProxyConnection, clientID uint32) *TranscoderRemoteIndi {
	return &TranscoderRemoteIndi{proxy_con, 0, true, 0, clientID}
}

func (t *TranscoderRemoteIndi) UpdateBitrate(bitrate uint32) {
	t.estimatedBitrate = uint32(float64(bitrate) * 0.9)
}

func (t *TranscoderRemoteIndi) UpdateProjection() {
	// Do nothing
}
func (t *TranscoderRemoteIndi) NextFrame() (uint32, []byte) {
	return proxyConn.NextFrame(t.clientID)
}

func (t *TranscoderRemoteIndi) EncodeFrame(data []byte, framecounter uint32, bitrate uint32) *Frame {
	if data == nil {
		return nil
	}
	rFrame := Frame{t.clientID, uint32(len(data)), framecounter, data}
	return &rFrame
}

func (t *TranscoderRemoteIndi) IsReady() bool {
	return t.isReady
}
func (t *TranscoderRemoteIndi) GetEstimatedBitrate() uint32 {
	return t.estimatedBitrate
}
func (t *TranscoderRemoteIndi) GetFrameCounter() uint32 {
	return t.frameCounter
}

type TranscoderDummy struct {
	proxy_con        *ProxyConnection
	frameCounter     uint32
	isReady          bool
	bitrate          uint32
	isFixed          bool
	isDummy          bool
	estimatedBitrate uint32
}

func NewTranscoderDummy(proxy_con *ProxyConnection, bitrate uint32, isFixed bool, isDummy bool) *TranscoderDummy {
	return &TranscoderDummy{proxy_con, 0, true, bitrate, isFixed, isDummy, 0}
}

func (t *TranscoderDummy) UpdateBitrate(bitrate uint32) {
	// Do nothing
	if !t.isFixed {
		t.bitrate = bitrate
	}

}

func (t *TranscoderDummy) UpdateProjection() {
	// Do nothing
}

func (t *TranscoderDummy) EncodeFrame(data []byte, framecounter uint32, bitrate uint32) *Frame {

	if t.isDummy {
		return nil
	}
	//	//println(100000 / 8 / t.n_tiles)
	transcodedData := make([]byte, uint32(float64(t.bitrate/8/30)))
	rFrame := Frame{0, uint32(len(transcodedData)), framecounter, transcodedData}
	t.frameCounter++
	return &rFrame
}

func (t *TranscoderDummy) IsReady() bool {
	return true
}
func (t *TranscoderDummy) GetEstimatedBitrate() uint32 {
	return t.estimatedBitrate
}
func (t *TranscoderDummy) GetFrameCounter() uint32 {
	return t.frameCounter
}
func (t *TranscoderDummy) NextFrame() (uint32, []byte) {
	return t.frameCounter, make([]byte, uint32(float64(t.bitrate/8/30)))
}
