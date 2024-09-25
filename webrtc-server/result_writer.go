package main

import (
	"fmt"
	"os"
)

type FrameResult struct {
	FrameNr                     uint32
	SizeInBytes                 uint32
	EntryTimestamp              int64
	ProcessingCompleteTimestamp int64
	Quality                     uint32
	EstimatedBitrate            uint32
	IsSender                    bool
}

func NewFrameResult(frameNr uint32, entryTimestamp int64, isSender bool) *FrameResult {
	return &FrameResult{
		FrameNr:        frameNr,
		EntryTimestamp: entryTimestamp,
		IsSender:       isSender,
	}
}

type FrameResultWriter struct {
	saveInterval uint32

	receivedFrames map[uint32]*FrameResult
	sendFrames     map[uint32]*FrameResult

	receivedFramesFile *os.File
	sendFramesFile     *os.File
}

func NewFrameResultWriter(path string, saveInterval uint32) *FrameResultWriter {
	fr := &FrameResultWriter{
		saveInterval:   saveInterval,
		receivedFrames: make(map[uint32]*FrameResult),
		sendFrames:     make(map[uint32]*FrameResult),
	}
	recvFile, err := os.OpenFile(path+"recv.csv", os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0644)
	if err != nil {
		panic("Error creating resultwriter file")
	}
	sendFile, err := os.OpenFile(path+"send.csv", os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0644)
	if err != nil {
		panic("Error creating resultwriter file")
	}
	fr.receivedFramesFile = recvFile
	fr.sendFramesFile = sendFile

	fr.receivedFramesFile.WriteString(fr.getHeader())
	fr.sendFramesFile.WriteString(fr.getHeader())
	return fr
}

func (fs *FrameResultWriter) CreateRecord(frameNr uint32, entryTimestamp int64, isSender bool) {
	fr := NewFrameResult(frameNr, entryTimestamp, isSender)
	if isSender {
		fs.sendFrames[frameNr] = fr
	} else {
		fs.receivedFrames[frameNr] = fr
	}
}

func (fs *FrameResultWriter) SetSizeInBytes(frameNr uint32, sizeInBytes uint32, isSender bool) {
	if isSender {
		if fr, ok := fs.sendFrames[frameNr]; ok {
			fr.SizeInBytes = sizeInBytes
		} else {
			//println("Setting size in bytes for unknown frame")
		}
	} else {
		if fr, ok := fs.receivedFrames[frameNr]; ok {
			fr.SizeInBytes = sizeInBytes
		} else {
			//println("Setting size in bytes for unknown frame")
		}
	}
}

func (fs *FrameResultWriter) SetQuality(frameNr uint32, quality uint32) {
	if fr, ok := fs.sendFrames[frameNr]; ok {
		fr.Quality = quality
	} else {
		//println("Setting quality for unknown frame")
	}
}

func (fs *FrameResultWriter) SetProcessingCompleteTimestamp(frameNr uint32, processingCompleteTimestamp int64, isSender bool) {
	if isSender {
		if fr, ok := fs.sendFrames[frameNr]; ok {
			fr.ProcessingCompleteTimestamp = processingCompleteTimestamp
		} else {
			//println("Setting processing complete timestamps for unknown frame")
		}
	} else {
		if fr, ok := fs.receivedFrames[frameNr]; ok {
			fr.ProcessingCompleteTimestamp = processingCompleteTimestamp
		} else {
			//println("Setting processing complete timestamps for unknown frame")
		}
	}
}

func (fs *FrameResultWriter) SetEstimatedBitrate(frameNr uint32, estimatedBitrate uint32) {
	if fr, ok := fs.sendFrames[frameNr]; ok {
		fr.EstimatedBitrate = estimatedBitrate
	} else {
		//println("Setting estimated bandwidth for unknown frame")
	}
}

func (fs *FrameResultWriter) SaveRecord(frameNr uint32, isSender bool) {
	if isSender {
		if fr, ok := fs.sendFrames[frameNr]; ok {
			if frameNr%fs.saveInterval == 0 {
				fs.sendFramesFile.WriteString(fs.getRecord(fr))
			}
			delete(fs.sendFrames, frameNr)
		} else {
			//println("SaveRecord unknown frame")
		}
	} else {
		if fr, ok := fs.receivedFrames[frameNr]; ok {
			if frameNr%fs.saveInterval == 0 {
				fs.receivedFramesFile.WriteString(fs.getRecord(fr))
			}
			delete(fs.receivedFrames, frameNr)
		} else {
			//println("SaveRecord unknown frame")
		}
	}

}

func (fs *FrameResultWriter) getHeader() string {
	return "frameNr;sizeInBytes;eTimestamp;pTimestamp;quality;estimatedBitrate;isSender;\n"
}

func (fs *FrameResultWriter) getRecord(fr *FrameResult) string {
	return fmt.Sprintf("%d;%d;%d;%d;%d;%d;%t\n", fr.FrameNr, fr.SizeInBytes, fr.EntryTimestamp, fr.ProcessingCompleteTimestamp, fr.Quality, fr.EstimatedBitrate, fr.IsSender)
}
