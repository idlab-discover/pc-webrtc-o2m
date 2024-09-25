package main

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"flag"
	"fmt"
	"net"
	"sync"
	"time"

	"github.com/gorilla/websocket"
	"github.com/pion/interceptor"
	"github.com/pion/interceptor/pkg/cc"
	"github.com/pion/interceptor/pkg/gcc"
	"github.com/pion/interceptor/pkg/nack"
	"github.com/pion/interceptor/pkg/twcc"
	"github.com/pion/webrtc/v3"
)

const (
	Idle     int = 0
	Hello    int = 1
	Offer    int = 2
	Answer   int = 3
	Ready    int = 4
	Finished int = 5
)

var clientCounter uint64
var proxyConn *ProxyConnection
var peerConnections map[uint64]*PeerConnection
var api *webrtc.API
var nClients int
var pcMapMutex sync.Mutex

// var frameResultwriter *FrameResultWriter
var virtualWallFilterIp string
var useProxy *bool
var isIndi *bool

func main() {
	virtualWallIp := flag.String("v", "", "Use virtual wall ip filter")
	useProxy = flag.Bool("p", false, "Use Proxy Input")
	capPort := flag.String("cap", ":8000", "Use as a proxy with specified port")
	srvPort := flag.String("srv", ":8001", "Use as a proxy with specified port")
	contentDirectory := flag.String("d", "content_jpg", "Content directory")
	contentFrameRate := flag.Int("f", 30, "Frame rate that is used when using files instead of proxy")
	signalingIP := flag.String("s", "0.0.0.0:5678", "Signaling server IP")
	numberOfClients := flag.Int("c", -1, "Number of clients")
	//resultDirectory := flag.String("m", "", "Result directory")
	isIndi = flag.Bool("i", false, "Use Individual Encoding")
	flag.Parse()
	//frameResultwriter = NewFrameResultWriter(*resultDirectory, 5)
	//fileCont, _ := os.OpenFile(*resultDirectory+"_cont.csv", os.O_WRONLY|os.O_CREATE|os.O_TRUNC, 0644)
	//fileCont.WriteString("time;estimated_bitrate;loss_rate;delay_rate;loss\n")
	nClients = *numberOfClients
	if *useProxy {
		proxyConn = NewProxyConnection(*isIndi)
		proxyConn.SetupConnection(*capPort, *srvPort)

	}
	var transcoder Transcoder
	if *useProxy {
		if !*isIndi {
			transcoder = NewTranscoderRemote(proxyConn)
		}
	} else {
		transcoder = NewTranscoderFile(*contentDirectory, uint32(*contentFrameRate))
	}
	// TODO Transcoder layered
	clientCounter = 0
	peerConnections = make(map[uint64]*PeerConnection)
	settingEngine := webrtc.SettingEngine{}
	settingEngine.SetSCTPMaxReceiveBufferSize(16 * 1024 * 1024)
	if *virtualWallIp != "" {
		virtualWallFilterIp = *virtualWallIp
		settingEngine.SetIPFilter(VirtualWallFilter)
	}
	i := &interceptor.Registry{}
	m := NewMediaEngine()
	// Sender side

	congestionController, err := cc.NewInterceptor(func() (cc.BandwidthEstimator, error) {
		return gcc.NewSendSideBWE(gcc.SendSideBWEMinBitrate(75_000*8), gcc.SendSideBWEInitialBitrate(15_000_000), gcc.SendSideBWEMaxBitrate(262_744_320))
	})
	if err != nil {
		panic(err)
	}

	congestionController.OnNewPeerConnection(func(id string, estimator cc.BandwidthEstimator) {
		println("NEW BW ESTIMATOR")
		peerConnections[clientCounter-1].estimator = estimator
	})

	i.Add(congestionController)
	if err = webrtc.ConfigureTWCCHeaderExtensionSender(m, i); err != nil {
		panic(err)
	}

	responder, _ := nack.NewResponderInterceptor()
	i.Add(responder)

	generator, err := twcc.NewSenderInterceptor(twcc.SendInterval(10 * time.Millisecond))
	if err != nil {
		panic(err)
	}

	i.Add(generator)

	nackGenerator, _ := nack.NewGeneratorInterceptor()
	i.Add(nackGenerator)
	api = webrtc.NewAPI(webrtc.WithSettingEngine(settingEngine), webrtc.WithInterceptorRegistry(i), webrtc.WithMediaEngine(m))
	//api = webrtc.NewAPI(webrtc.WithSettingEngine(settingEngine), webrtc.WithInterceptorRegistry(i), webrtc.WithMediaEngine(m))
	//api = NewWebrtcAPI(peerConnections)

	defer func() {
		// CLOSE ALL PEER CONNECTIONS
		for _, p := range peerConnections {
			if cErr := p.webrtcConnection.Close(); cErr != nil {
				fmt.Printf("Cannot close peer connection: %v\n", cErr)
			}
		}
	}()
	// CHANGE TO SERVER

	var state = Idle
	println("Current state:", state)
	NewWSServer(*signalingIP, wsNewUserCb)
	// Infinite loop sending aggregate frames every 33ms

	//select {}
	if !*isIndi {
		for {
			frameNr, frame := transcoder.NextFrame()
			pcMapMutex.Lock()
			for _, pc := range peerConnections {
				// Get frame from proxy = channel (maybe ring channel)
				if pc.isReady {
					go pc.SendFrame(transcoder.EncodeFrame(frame, frameNr, pc.GetBitrate()))
				}
			}
			pcMapMutex.Unlock()
		}
	} else {
		select {}
	}
}
func getCodecCapability() webrtc.RTPCodecCapability {
	videoRTCPFeedback := []webrtc.RTCPFeedback{
		{Type: "goog-remb", Parameter: ""},
		{Type: "ccm", Parameter: "fir"},
		{Type: "nack", Parameter: ""},
		{Type: "nack", Parameter: "pli"},
	}

	return webrtc.RTPCodecCapability{
		MimeType:     "video/pcm",
		ClockRate:    90000,
		Channels:     0,
		SDPFmtpLine:  "",
		RTCPFeedback: videoRTCPFeedback,
	}
}
func NewMediaEngine() *webrtc.MediaEngine {
	m := &webrtc.MediaEngine{}
	if err := m.RegisterDefaultCodecs(); err != nil {
		panic(err)
	}
	if err := m.RegisterCodec(webrtc.RTPCodecParameters{
		RTPCodecCapability: getCodecCapability(),
		PayloadType:        5,
	}, webrtc.RTPCodecTypeVideo); err != nil {
		panic(err)
	}
	m.RegisterFeedback(webrtc.RTCPFeedback{Type: "nack"}, webrtc.RTPCodecTypeVideo)
	m.RegisterFeedback(webrtc.RTCPFeedback{Type: "nack", Parameter: "pli"}, webrtc.RTPCodecTypeVideo)
	//m.RegisterFeedback(webrtc.RTCPFeedback{Type: webrtc.TypeRTCPFBTransportCC}, webrtc.RTPCodecTypeVideo)
	/*if err := m.RegisterHeaderExtension(webrtc.RTPHeaderExtensionCapability{URI: sdp.TransportCCURI}, webrtc.RTPCodecTypeVideo); err != nil {
		panic(err)
	}

	m.RegisterFeedback(webrtc.RTCPFeedback{Type: webrtc.TypeRTCPFBTransportCC}, webrtc.RTPCodecTypeAudio)
	if err := m.RegisterHeaderExtension(webrtc.RTPHeaderExtensionCapability{URI: sdp.TransportCCURI}, webrtc.RTPCodecTypeAudio); err != nil {
		panic(err)
	}*/
	return m
}

func wsNewUserCb(wsConn *websocket.Conn) {
	pcMapMutex.Lock()
	defer pcMapMutex.Unlock()
	fmt.Printf("New Websocket user ID: %d\n", clientCounter)
	peerConnections[clientCounter] = NewPeerConnection(clientCounter, wsConn, wsHandlerMessageCbFunc, *isIndi)
	peerConnections[clientCounter].SetOnConnectedCb(OnPeerConnected)
	peerConnections[clientCounter].SetOnDisconnectedCb(OnPeerDisconnected)
	peerConnections[clientCounter].Init()
	clientCounter++
	//if int(clientCounter) == nClients {
	//	proxyConn.StartListening()
	//}
}

func wsHandlerMessageCbFunc(wsPacket WebsocketPacket, pc *PeerConnection) {
	switch wsPacket.MessageType {
	case 3: // answer
		answer := webrtc.SessionDescription{}
		if err := json.Unmarshal([]byte(wsPacket.Message), &answer); err != nil {
			panic(err)
		}
		if err := pc.SetRemoteDescription(answer); err != nil {
			panic(err)
		}
		for _, c := range pc.pendingCandidatesString {
			if candidateErr := pc.AddICECandidate(c); candidateErr != nil {
				panic(candidateErr)
			}
		}
	case 4: // candidate
		desc := pc.GetRemoteDescription()
		if desc == nil {
			pc.pendingCandidatesString = append(pc.pendingCandidatesString, wsPacket.Message)
		} else {
			if candidateErr := pc.AddICECandidate(wsPacket.Message); candidateErr != nil {
				panic(candidateErr)
			}
		}
	case 10: //panzoom TODO rework
		bufBinary := bytes.NewBuffer([]byte(wsPacket.Message))
		var pz PanZoom
		if err := binary.Read(bufBinary, binary.LittleEndian, &pz); err == nil {
			peerConnections[wsPacket.ClientID].SetPanZoom(pz)
		}
	default:
		println(fmt.Sprintf("Received non-compliant message type %d", wsPacket.MessageType))
	}
}

func VirtualWallFilter(addr net.IP) bool {
	if addr.String() == virtualWallFilterIp {
		return true
	}
	return false
}

func OnPeerConnected(clientID uint64) {
	if *useProxy {
		proxyConn.OnNewClientConnected(uint32(clientID))
	}
}

func OnPeerDisconnected(clientID uint64) {
	pcMapMutex.Lock()
	defer pcMapMutex.Unlock()
	delete(peerConnections, clientID)
	if *useProxy {
		proxyConn.OnNewClientDisconnected(uint32(clientID))
	}
}
