package main

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"log"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/gorilla/websocket"
	"github.com/pion/interceptor"
	"github.com/pion/interceptor/pkg/cc"
	"github.com/pion/interceptor/pkg/gcc"
	"github.com/pion/interceptor/pkg/nack"
	"github.com/pion/interceptor/pkg/twcc"
	"github.com/pion/sdp/v3"
	"github.com/pion/webrtc/v3"
)

type PanZoom struct {
	XPos float32
	YPos float32
	ZPos float32

	XRot float32
	YRot float32
	ZRot float32
}

type SendMessageCallback func(WebsocketPacket)
type OnDisconnectedCb func(uint64)
type OnConnectedCb func(uint64)

type PeerConnectionFrame struct {
	ClientID   uint64
	FrameNr    uint32
	FrameLen   uint32
	CurrentLen uint32
	FrameData  []byte
}

func NewPeerConnectionFrame(clientID uint64, frameNr uint32, frameLen uint32) *PeerConnectionFrame {
	return &PeerConnectionFrame{clientID, frameNr, frameLen, 0, make([]byte, frameLen)}
}

func (pf *PeerConnectionFrame) IsComplete() bool {
	return pf.CurrentLen == pf.FrameLen
}

// TODO State variable per connection
// TODO Frame queue per connection
type PeerConnection struct {
	websocketConnection     *websocket.Conn
	wbMutex                 sync.Mutex
	webrtcConnection        *webrtc.PeerConnection
	clientID                uint64
	candidatesMux           sync.Mutex
	pendingCandidates       []*webrtc.ICECandidate
	pendingCandidatesString []string
	estimator               cc.BandwidthEstimator
	track                   *TrackLocalCloudRTP
	transcoder              Transcoder
	isIndi                  bool

	frames                 map[uint32]*PeerConnectionFrame
	completedFramesChannel *RingChannel
	isReady                bool

	panZoomMux     sync.Mutex
	currentPanZoom PanZoom

	frameResultWriter FrameResultWriter
	currentFrameNr    uint64

	conCb OnConnectedCb
	dscCb OnDisconnectedCb
}

// TODO add offer parameter?
func NewPeerConnection(clientID uint64, websocketConnection *websocket.Conn, wsCb WebsocketCallback, isIndi bool) *PeerConnection {
	// TODO Make new webrtc connection
	// TODO Error checking
	pc := &PeerConnection{
		websocketConnection:     websocketConnection,
		wbMutex:                 sync.Mutex{},
		clientID:                clientID,
		candidatesMux:           sync.Mutex{},
		pendingCandidates:       make([]*webrtc.ICECandidate, 0),
		pendingCandidatesString: make([]string, 0),
		frames:                  make(map[uint32]*PeerConnectionFrame),
		completedFramesChannel:  NewRingChannel(100),
		frameResultWriter:       *NewFrameResultWriter(strconv.Itoa(int(clientID)), 5),
		currentFrameNr:          0,
		isIndi:                  isIndi,
	}
	if isIndi {
		pc.transcoder = NewTranscoderRemoteIndi(proxyConn, uint32(clientID))
	}
	pc.StartListeningWebsocket(wsCb)
	return pc
}

func (pc *PeerConnection) NewWebrtcAPI() *webrtc.API {
	settingEngine := webrtc.SettingEngine{}
	settingEngine.SetSCTPMaxReceiveBufferSize(16 * 1024 * 1024)

	i := &interceptor.Registry{}
	m := &webrtc.MediaEngine{}
	if err := m.RegisterDefaultCodecs(); err != nil {
		panic(err)
	}

	videoRTCPFeedback := []webrtc.RTCPFeedback{
		{Type: "goog-remb", Parameter: ""},
		{Type: "ccm", Parameter: "fir"},
		{Type: "nack", Parameter: ""},
		{Type: "nack", Parameter: "pli"},
	}

	codecCapability := webrtc.RTPCodecCapability{
		MimeType:     "video/pcm",
		ClockRate:    90000,
		Channels:     0,
		SDPFmtpLine:  "",
		RTCPFeedback: videoRTCPFeedback,
	}

	if err := m.RegisterCodec(webrtc.RTPCodecParameters{
		RTPCodecCapability: codecCapability,
		PayloadType:        5,
	}, webrtc.RTPCodecTypeVideo); err != nil {
		panic(err)
	}

	m.RegisterFeedback(webrtc.RTCPFeedback{Type: "nack"}, webrtc.RTPCodecTypeVideo)
	m.RegisterFeedback(webrtc.RTCPFeedback{Type: "nack", Parameter: "pli"}, webrtc.RTPCodecTypeVideo)
	// Sender side
	congestionController, err := cc.NewInterceptor(func() (cc.BandwidthEstimator, error) {
		return gcc.NewSendSideBWE(gcc.SendSideBWEMinBitrate(75_000*8), gcc.SendSideBWEInitialBitrate(75_000_000), gcc.SendSideBWEMaxBitrate(262_744_320))
	})
	if err != nil {
		panic(err)
	}

	congestionController.OnNewPeerConnection(func(id string, estimator cc.BandwidthEstimator) {
		pc.SetEstimator(estimator)
	})
	m.RegisterFeedback(webrtc.RTCPFeedback{Type: webrtc.TypeRTCPFBTransportCC}, webrtc.RTPCodecTypeVideo)
	if err := m.RegisterHeaderExtension(webrtc.RTPHeaderExtensionCapability{URI: sdp.TransportCCURI}, webrtc.RTPCodecTypeVideo); err != nil {
		panic(err)
	}
	//webrtc.RegisterDefaultInterceptors()
	responder, _ := nack.NewResponderInterceptor()
	twccInt, _ := twcc.NewHeaderExtensionInterceptor()
	generator, err := twcc.NewSenderInterceptor(twcc.SendInterval(10 * time.Millisecond))
	if err != nil {
		panic(err)
	}

	nackGenerator, _ := nack.NewGeneratorInterceptor()

	i.Add(congestionController)
	i.Add(responder)
	i.Add(twccInt)
	i.Add(generator)
	i.Add(nackGenerator)

	return webrtc.NewAPI(webrtc.WithSettingEngine(settingEngine), webrtc.WithInterceptorRegistry(i), webrtc.WithMediaEngine(m))
}

func (pc *PeerConnection) Init() {
	api := pc.NewWebrtcAPI()
	webrtcConnection, _ := api.NewPeerConnection(webrtc.Configuration{})
	pc.webrtcConnection = webrtcConnection
	// ------------------ Callbacks ------------------
	webrtcConnection.OnICECandidate(pc.OnIceCandidateCb)
	webrtcConnection.OnConnectionStateChange(pc.OnConnectionStateChangeCb)
	webrtcConnection.OnTrack(pc.OnTrackCb)
	// -----------------------------------------------
	codecCap := getCodecCapability()
	codecCap.RTCPFeedback = nil
	videoTrack, err := NewTrackLocalCloudRTP(codecCap, "video", "pion")
	if err != nil {
		panic(err)
	}
	pc.track = videoTrack
	// RTP Sender
	rtpSender, err := webrtcConnection.AddTrack(videoTrack)
	if err != nil {
		panic(err)
	}
	go func() {
		rtcpBuf := make([]byte, 1500)
		for {
			if _, _, err := rtpSender.Read(rtcpBuf); err != nil {
				panic(err)
			}
		}
	}()
	offer, err := pc.webrtcConnection.CreateOffer(nil)
	if err != nil {

	}

	if err = pc.webrtcConnection.SetLocalDescription(offer); err != nil {

	}

	payload, err := json.Marshal(offer)

	pc.SendWebsocketMessage(WebsocketPacket{0, 2, string(payload)})
	// ------------------ Set Description ------------------
	/*offer, err := webrtcConnection.CreateOffer(nil)
	webrtcConnection.SetLocalDescription(offer)
	payload, err := json.Marshal(offer)
	if err != nil {
		panic(err)
	}
	pc.SendWebsocketMessage(WebsocketPacket{uint64(pc.clientID), 2, string(payload)})*/
	/*webrtcConnection.SetRemoteDescription(offer)
	answer, err := webrtcConnection.CreateAnswer(nil)
	if err != nil {
		panic(err)
	}
	if err = webrtcConnection.SetLocalDescription(answer); err != nil {
		panic(err)
	}
	payload, err := json.Marshal(answer)
	if err != nil {
		panic(err)
	}
	pc.SendWebsocketMessage(WebsocketPacket{uint64(pc.clientID), 3, string(payload)})*/
	// -----------------------------------------------------
}

func (pc *PeerConnection) SetRemoteDescription(answer webrtc.SessionDescription) error {
	pc.webrtcConnection.SetRemoteDescription(answer)
	pc.candidatesMux.Lock()
	for _, c := range pc.pendingCandidates {
		payload := []byte(c.ToJSON().Candidate)
		pc.SendWebsocketMessage(WebsocketPacket{1, 4, string(payload)})
	}
	pc.candidatesMux.Unlock()
	return nil
}

func (pc *PeerConnection) AddICECandidate(candidate string) error {
	return pc.webrtcConnection.AddICECandidate(webrtc.ICECandidateInit{Candidate: candidate})
}

func (pc *PeerConnection) SetEstimator(estimator cc.BandwidthEstimator) {
	pc.estimator = estimator
}
func (pc *PeerConnection) StartListeningWebsocket(wsCb WebsocketCallback) {
	go func() {
		for {
			_, message, err := pc.websocketConnection.ReadMessage()
			if err != nil {
				log.Println("read:", err)
				break
			}
			v := strings.Split(string(message), "@")
			messageType, _ := strconv.ParseUint(v[1], 10, 64)
			wsPacket := WebsocketPacket{uint64(pc.clientID), messageType, v[2]}
			// TODO Potential clash => adding new client => currently reading from it
			// Complete peer connection initilisation
			wsCb(wsPacket, pc)
		}
	}()
}
func (pc *PeerConnection) SendWebsocketMessage(wsPacket WebsocketPacket) {
	s := fmt.Sprintf("%d@%d@%s", wsPacket.ClientID, wsPacket.MessageType, wsPacket.Message)
	pc.wbMutex.Lock()
	defer pc.wbMutex.Unlock()
	err := pc.websocketConnection.WriteMessage(websocket.TextMessage, []byte(s))
	if err != nil {
		panic(err)
	}
}

// TODO Pass global wsHandler?
func (pc *PeerConnection) SetOnConnectedCb(cb OnConnectedCb) {
	pc.conCb = cb
}
func (pc *PeerConnection) SetOnDisconnectedCb(cb OnDisconnectedCb) {
	pc.dscCb = cb
}

// TODO Pass global wsHandler?
func (pc *PeerConnection) OnIceCandidateCb(c *webrtc.ICECandidate) {
	if c == nil {
		return
	}
	pc.candidatesMux.Lock()
	desc := pc.webrtcConnection.RemoteDescription()
	if desc == nil {
		pc.pendingCandidates = append(pc.pendingCandidates, c)
	} else {
		payload := []byte(c.ToJSON().Candidate)
		// TODO WS HANDLER
		pc.SendWebsocketMessage(WebsocketPacket{uint64(pc.clientID), 4, string(payload)})
	}
	pc.candidatesMux.Unlock()
}

// TODO Change implentation => add connection to completed clients
func (pc *PeerConnection) OnConnectionStateChangeCb(s webrtc.PeerConnectionState) {
	fmt.Printf("Peer connection state has changed: %s\n", s.String())
	if s == webrtc.PeerConnectionStateFailed {
		fmt.Println("Peer connection has gone to failed exiting")
		os.Exit(0)
	} else if s == webrtc.PeerConnectionStateConnected {
		pc.isReady = true
		if pc.conCb != nil {
			println("concb", pc.clientID)
			pc.conCb(pc.clientID)
		}
		if pc.isIndi {
			go func() {
				for {
					frameNr, frame := pc.transcoder.NextFrame()
					pc.SendFrame(pc.transcoder.EncodeFrame(frame, frameNr, pc.GetBitrate()))
				}
			}()
		}

	} else if s == webrtc.PeerConnectionStateClosed {
		if pc.dscCb != nil {
			pc.dscCb(pc.clientID)
		}
	}
}

// Parameter => connection ID
func (pc *PeerConnection) OnTrackCb(track *webrtc.TrackRemote, receiver *webrtc.RTPReceiver) {

	println("OnTrack has been called")
	println("MIME type:", track.Codec().MimeType)
	println("Payload type:", track.PayloadType())

	codecName := strings.Split(track.Codec().RTPCodecCapability.MimeType, "/")
	fmt.Printf("Track of type %d has started: %s \n", track.PayloadType(), codecName)

	// Create buffer to receive incoming track data, using 1300 bytes - header bytes
	buf := make([]byte, 1220)

	// Allows to check if frames are received completely
	// Frame number and corresponding length
	for {
		_, _, readErr := track.Read(buf)
		if readErr != nil {
			panic(readErr)
		}
		// Create a buffer from the byte array, skipping the first 20 WebRTC bytes
		// TODO: mention WebRTC header content explicitly
		bufBinary := bytes.NewBuffer(buf[20:])
		// Read the fields from the buffer into a struct
		var p FramePacket
		err := binary.Read(bufBinary, binary.LittleEndian, &p)
		if err != nil {
			panic(err)
		}
		var frame *PeerConnectionFrame
		var ok bool
		if frame, ok = pc.frames[p.FrameNr]; !ok {
			frame = NewPeerConnectionFrame(pc.clientID, p.FrameNr, p.FrameLen)
			pc.frames[p.FrameNr] = frame
		}
		copy(frame.FrameData[frame.CurrentLen:], p.Data[:])
		frame.CurrentLen += p.SeqLen
		if frame.IsComplete() {
			if frame.FrameNr%100 == 0 {
				println("FRAME COMPLETE ", pc.clientID, p.FrameNr, p.FrameLen)
			}
			// Will drop oldest frame if capacity is full
			pc.completedFramesChannel.In() <- frame
			delete(pc.frames, p.FrameNr)
		}
	}

}

func (pc *PeerConnection) GetBitrate() uint32 {
	return uint32(pc.estimator.GetTargetBitrate())
}

func (pc *PeerConnection) GetFrameCounter() uint32 {
	return uint32(pc.currentFrameNr)
}

func (pc *PeerConnection) GetPanZoom() PanZoom {
	pc.panZoomMux.Lock()
	defer pc.panZoomMux.Unlock()
	return pc.currentPanZoom
}

func (pc *PeerConnection) GetRemoteDescription() *webrtc.SessionDescription {
	return pc.webrtcConnection.RemoteDescription()
}

func (pc *PeerConnection) SetPanZoom(pz PanZoom) {
	pc.panZoomMux.Lock()
	defer pc.panZoomMux.Unlock()
	pc.currentPanZoom = pz
}

func (pc *PeerConnection) SendFrame(frame *Frame) {
	if frame != nil {
		pc.frameResultWriter.CreateRecord(uint32(frame.FrameNr), time.Now().UnixNano()/int64(time.Millisecond), true)
		pc.frameResultWriter.SetEstimatedBitrate(uint32(frame.FrameNr), uint32(pc.estimator.GetTargetBitrate()))
		pc.frameResultWriter.SetSizeInBytes(uint32(frame.FrameNr), frame.FrameLen, true)

		pc.track.WriteFrame(frame)
		if frame.FrameNr%100 == 0 {
			println("MULTIFRAME", frame.FrameNr, pc.clientID, len(frame.Data))
		}

		pc.frameResultWriter.SetProcessingCompleteTimestamp(uint32(frame.FrameNr), time.Now().UnixNano()/int64(time.Millisecond), true)
		pc.frameResultWriter.SaveRecord(uint32(frame.FrameNr), true)
	}
	//pc.currentFrameNr++
}

func (pc *PeerConnection) EncodeFrame(l0 []byte, l1 []byte, l2 []byte) *Frame {

	//transcodedData := t.lEnc.EncodeMultiFrame(data)
	tempBitrate := (pc.estimator.GetTargetBitrate()) / 8 / 30
	fileData := make([]byte, 0)

	l0Size := len(l0)
	l1Size := len(l1)
	l2Size := len(l2)
	if tempBitrate >= l0Size {
		tempBitrate -= l0Size
		fileData = append(fileData, l0...)
		if tempBitrate >= l1Size {
			tempBitrate -= l1Size
			fileData = append(fileData, l1...)
			if tempBitrate >= l2Size {
				tempBitrate -= l2Size
				fileData = append(fileData, l2...)
			}
		} else if tempBitrate >= l2Size {
			tempBitrate -= l2Size
			fileData = append(fileData, l2...)
		}
	} else if tempBitrate >= l1Size {
		tempBitrate -= l1Size
		fileData = append(fileData, l1...)
		if tempBitrate >= l2Size {
			tempBitrate -= l2Size
			fileData = append(fileData, l2...)
		}
	} else if tempBitrate >= l2Size {
		tempBitrate -= l2Size
		fileData = append(fileData, l2...)
	}
	if uint32(len(fileData)) == 0 {
		return nil
	}
	rFrame := Frame{0, uint32(len(fileData)), uint32(pc.currentFrameNr), fileData}
	return &rFrame
}
