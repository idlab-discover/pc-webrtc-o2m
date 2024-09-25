package main

import (
	"github.com/pion/rtp"
	"github.com/pion/webrtc/v3"
)

// TrackLocalStaticRTP  is a TrackLocal that has a pre-set codec and accepts RTP Packets.
// If you wish to send a media.Sample use TrackLocalStaticSample
type TrackLocalCloudRTP struct {
	packetizer   rtp.Packetizer
	sequencer    rtp.Sequencer
	rtpTrack     *webrtc.TrackLocalStaticRTP
	clockRate    float64
	currentFrame uint32
	pcPayloader  *PointCloudPayloader
}

// NewTrackLocalStaticSample returns a TrackLocalStaticSample
func NewTrackLocalCloudRTP(c webrtc.RTPCodecCapability, id, streamID string, options ...func(*webrtc.TrackLocalStaticRTP)) (*TrackLocalCloudRTP, error) {
	rtpTrack, err := webrtc.NewTrackLocalStaticRTP(c, id, streamID, options...)
	if err != nil {
		return nil, err
	}
	return &TrackLocalCloudRTP{
		rtpTrack: rtpTrack,
	}, nil
}

// Bind is called by the PeerConnection after negotiation is complete
// This asserts that the code requested is supported by the remote peer.
// If so it setups all the state (SSRC and PayloadType) to have a call
func (s *TrackLocalCloudRTP) Bind(t webrtc.TrackLocalContext) (webrtc.RTPCodecParameters, error) {
	codec, err := s.rtpTrack.Bind(t)
	if err != nil {
		return codec, err
	}

	// We only need one packetizer
	if s.packetizer != nil {
		return codec, nil
	}
	s.pcPayloader = NewPointCloudPayloader()
	s.sequencer = rtp.NewRandomSequencer()
	s.packetizer = rtp.NewPacketizer(
		1200, // Not MTU but ok
		0,    // Value is handled when writing
		0,    // Value is handled when writing
		s.pcPayloader,
		s.sequencer,
		codec.ClockRate,
	)

	s.clockRate = float64(codec.RTPCodecCapability.ClockRate)
	return codec, err
}

// Unbind implements the teardown logic when the track is no longer needed. This happens
// because a track has been stopped.
func (s *TrackLocalCloudRTP) Unbind(t webrtc.TrackLocalContext) error {
	return s.rtpTrack.Unbind(t)
}

// ID is the unique identifier for this Track. This should be unique for the
// stream, but doesn't have to globally unique. A common example would be 'audio' or 'video'
// and StreamID would be 'desktop' or 'webcam'
func (s *TrackLocalCloudRTP) ID() string { return s.rtpTrack.ID() }

// StreamID is the group this track belongs too. This must be unique
func (s *TrackLocalCloudRTP) StreamID() string { return s.rtpTrack.StreamID() }

// RID is the RTP stream identifier
func (s *TrackLocalCloudRTP) RID() string { return s.rtpTrack.RID() }

// Kind controls if this TrackLocal is audio or video
func (s *TrackLocalCloudRTP) Kind() webrtc.RTPCodecType { return s.rtpTrack.Kind() }

// Codec gets the Codec of the track
func (s *TrackLocalCloudRTP) Codec() webrtc.RTPCodecCapability {
	return s.rtpTrack.Codec()
}

func (s *TrackLocalCloudRTP) WriteFrame(frame *Frame) error {
	p := s.packetizer
	clockRate := s.clockRate

	if p == nil {
		return nil
	}

	samples := uint32(1 * clockRate)
	//frameData := t.EncodeFrame()
	s.pcPayloader.FrameCounter = frame.FrameNr
	packets := p.Packetize(frame.Data, samples)

	writeErrs := []error{}
	counter := 0
	for _, p := range packets {
		if err := s.rtpTrack.WriteRTP(p); err != nil {
			writeErrs = append(writeErrs, err)
		}
		counter += 1
	}
	if len(writeErrs) > 0 {
		//println(writeErrs)
	}

	return nil
}
