// dummyBATMAN.go
// send commands over BM network

// pi3: go run dummyBATMAN.go -rasp-id=66 --web-addr :8082 -log-level debug
// pi4: go run dummyBATMAN.go -rasp-id=67 --web-addr :8082 -log-level debug

// push from 4->3:
// rsync -a dummyBATMAN.go pi@192.168.1.166:code/go/src/github.com/johnusher/priWand/

package main

import (
	"flag"
	"fmt"

	// "math/rand"
	"net"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/johnusher/priWand/pkg/iface"
	"github.com/johnusher/priWand/pkg/keyboard"

	// "github.com/johnusher/priWand/pkg/lcd"

	"github.com/johnusher/priWand/pkg/readBATMAN"
	"github.com/johnusher/priWand/pkg/web"

	log "github.com/sirupsen/logrus"
)

const (
	batPort = 4200
	// msgSize   = net.IPv4len + 4 // IP + uint32
	interval  = 1 * time.Second
	ifaceName = "bat0" // rpi
	// ifaceName = "en0" // pc

	batBcast   = "172.27.255.255"
	localBcast = "127.0.0.1"

	// piTTL defines how long we wait to expire a PI if we haven't received a
	// message from it.
	piTTL = 30 * time.Second

	Pi = 3.14159265358979323846264338327950288419716939937510582097494459 // pi https://oeis.org/A000796

	magicByte = ("BA")

	messageTypeGPS    = 0
	messageTypeDuino  = 1
	messageTypeKey    = 2
	messageTypeButton = 3
	messageTypeAck    = 4
	messageTypeHelloA = 5
	messageTypeHelloR = 6
	raspiIDEveryone   = "00"
)

// ChatRequest is ChatRequest, stop telling me about comments
type ChatRequest struct {
	Latf         float64
	Longf        float64
	HDOPf        float64
	ID           string
	Key          rune
	PointDir     int64
	ButtonStatus int64
	ShieldTimer  time.Time
}

type chatRequestWithTimestamp struct {
	ChatRequest
	lastMessageReceived time.Time
}

var allPIs = map[string]chatRequestWithTimestamp{} // how to make this readable by broadcastLoop??

var OtherID = "bob"

// String satisfies the Stringer interface
func (c ChatRequest) String() string {
	// return fmt.Sprintf("id: %s, coords: (%f, %f), HDOP: %.2f", c.ID, c.Latf, c.Longf, c.HDOPf)
	return fmt.Sprintf("id: %s", c.ID)
}

// String satisfies the Stringer interface
func (c chatRequestWithTimestamp) String() string {
	return fmt.Sprintf("%s, age: %s", c.ChatRequest, time.Now().Sub(c.lastMessageReceived))
}

// // ShieldTimer
// func (c ChatRequestST) String() string {
// 	return fmt.Sprintf("ShieldTimer: %s", c.ShieldTimer)
// }

func main() {
	raspID := flag.String("rasp-id", "60", "unique raspberry pi ID") // only 2 characters. use last 2 digits of IP
	webAddr := flag.String("web-addr", ":8080", "address to serve web on")
	noBatman := flag.Bool("no-batman", false, "run without batman network")

	logLevel := flag.String("log-level", "info", "log level, must be one of: panic, fatal, error, warn, info, debug, trace")

	flag.Parse()

	level, err := log.ParseLevel(*logLevel)
	if err != nil {
		log.Errorf("failed to parse log level [%s]: %s", *logLevel, err)
		return
	}
	log.SetLevel(level)

	// make raspID into 2 bytes: take first 2 letter if needed:

	*raspID = (*raspID)[0:2]

	// now start web broadcast

	web := web.InitWeb(*webAddr)
	log.Infof("web: %+v", web)

	bcastIP := net.ParseIP(batBcast)
	if *noBatman {
		bcastIP = net.ParseIP(localBcast)
	}

	// Setup keyboard input:
	stop := make(chan os.Signal, 1)
	signal.Notify(stop, os.Interrupt, syscall.SIGTERM)

	keys := make(chan rune)

	kb, err := keyboard.Init(keys)
	if err != nil {
		log.Errorf("failed to initialize keyboard: %s", err)
		return
	}

	//  now setup BATMAN:
	// TODO: move all this to a function called "setUpBATMAN"

	myIP := net.IP{}

	i, err := iface.InterfaceByName(ifaceName, *noBatman, bcastIP)
	if err != nil {
		log.Errorf("InterfaceByName failed: %s", err)
		return
	}

	addrs, err := i.Addrs()
	if err != nil {
		log.Errorf("Failed to get addresses for interface %+v: %s", i, err)
		return
	}

	for _, addr := range addrs {
		ipnet := addr.(*net.IPNet)
		ip4 := ipnet.IP.To4()
		if ip4 != nil && ip4[0] == bcastIP.To4()[0] {
			myIP = ip4
		}
	}

	log.Infof("Serving at %s", myIP)

	// init BATMAN:
	messages := make(chan []byte)
	bm, err := readBATMAN.Init(messages, *noBatman, bcastIP)
	if err != nil {
		log.Errorf("failed to initialize readBATMAN: %s", err)
		return
	}

	// go forth
	go kb.Run()
	go bm.Run()

	// now we run the two loops
	// receiveBATMAN: listen for incoming, eg button press or messages on the BATMAN
	// broadcastLoop: send message to BATMAN
	errs := make(chan error)

	go func() {
		errs <- receiveBATMAN(messages, *raspID, web, bcastIP, bm)
	}()
	go func() {
		errs <- broadcastLoop(keys, *raspID, bcastIP, bm)
	}()
	go func() {
		// handle key presses from web, send to messages channel
		for {
			phoneEvent, more := <-web.Phone()
			if !more {
				log.Errorf("web phoneEvent channel closed")
				return
			}

			if len(phoneEvent.Key) == 0 {
				continue
			}

			keys <- []rune(phoneEvent.Key)[0]
		}
	}()

	// block until ctrl-c or one of the loops returns an error
	select {
	case <-stop:
	case <-errs:
	}
}

// receiveBATMAN receives incoming messages
// 2 bytes: <2 magic bytes>
// 1 byte:  <total message length, bytes>
// 2 bytes: <sender ID = 2 bytes, (IP?)>
// 2 bytes: <who For = 2 bytes, (0= everyone, or ID of)>
// 1 byte:  <message type (0=gps, 1=duino command, 2=gesture type)>
// N bytes: <message, >0 bytes>
func receiveBATMAN(messages <-chan []byte, raspID string, web *web.Web, bcastIP net.IP, bm *readBATMAN.ReadBATMAN) error {

	log.Info("Starting message loop")
	// listen for new incoming BATMAN message
	// allPIs keeps track of the last message received from each PI, keyed by
	// raspID

	// allPIs := map[string]chatRequestWithTimestamp{}  // how to make this readable by broadcastLoop??

	// accMessage := acc.ACCMessage{}
	bcast := &net.UDPAddr{Port: batPort, IP: bcastIP}

	crwt, _ := allPIs[raspID]
	crwt.ButtonStatus = 0 // init button status to button up: for some reason this should be a 1 for up but ...

	// crwt.ShieldTimer = time.Now() // reset shield timer

	// more := false

	for {
		select {

		case message, _ := <-messages:
			// message on the BATMAN

			magicBytesRx := string(message[0:2]) // combine the magicBytes
			if magicBytesRx != magicByte {
				log.Errorf("magicBytesRx error: Received %s, expected %s", string(magicBytesRx), magicByte)
				continue
			}

			// messageLength := uint8(message[2])  // todo: check message is correct length!
			senderID := string(message[3:5]) // this is length of raspID = 2 bytes
			whoFor := string(message[5:7])   // whoFor is also length of raspID = 2 bytes
			messageType := message[7]

			log.Infof("senderID %s \n", senderID)

			if senderID == raspID {
				// from self: ignore

			} else {

				OtherID = senderID // set other ID

				if messageType == messageTypeHelloA {
					log.Infof("hello from someone else!\n")

					// send hello back
					buttonmsgSize := 9                         // 32(?) bytes for  a hello message
					bMessageOut := make([]byte, buttonmsgSize) // sent to batman
					copy(bMessageOut[0:2], magicByte)
					bMessageOut[2] = uint8(buttonmsgSize)
					copy(bMessageOut[3:5], raspID)

					whoFor := raspiIDEveryone // message for everyone
					copy(bMessageOut[5:7], whoFor)
					bMessageOut[7] = uint8(messageTypeHelloR) // messageTypeHelloR reply

					bMessageOut[8] = uint8(5)
					_, err := bm.Conn.WriteToUDP(bMessageOut, bcast)
					if err != nil {
						log.Error(err)
						return err
					}

				}

				if (whoFor == raspiIDEveryone || whoFor == raspID) && (messageType != messageTypeAck) { // if message[6] == 0 || whoFor == raspID { // message[6] == 0  means for everyone.
					// message is not sent by self and is for everyone or for me

					if messageType == messageTypeKey {

						// first unpack the message:
						keyMessagek := message[8] // we should maybe look at total message legnth and combine other bytes if longer than 7

						log.Infof("key from other %s \n", (string(keyMessagek)))

						if (string(keyMessagek) == "c") || (string(keyMessagek) == "C") {
							// check for shielf active
							log.Infof("C received\n")

							now := time.Now()
							elapsedTime := now.Sub(crwt.ShieldTimer)
							log.Infof("shield active for: %v\n", elapsedTime)

							// crwt.ShieldTimer = now // reset shield timer:
							// crwt.ChatRequest.PointDir := 1.0 // reset shield timer: problem here!!
						}

						// now send an ack back to them

						// send just to the one we are pointing at:

						// send key=1 to network, to the piID. ie using broadcastLoop

						// ack message (9 bytes)
						// 2 bytes: <2 magic bytes>
						// 1 byte:  <total message length, bytes>
						// 2 bytes: <sender ID = 2 bytes, (IP?)>
						// 2 bytes: <who For = 2 bytes, (0= everyone, or ID of)>
						// 1 byte:  <message type (0=gps, 1=duino command, 2=gesture type, 4 = ack)>
						// 1 byte:  key

						duinoMsgSize := 9                        //  bytes for a duino message
						messageOut := make([]byte, duinoMsgSize) // sent to batman

						copy(messageOut[0:2], magicByte)
						messageOut[2] = uint8(duinoMsgSize)
						copy(messageOut[3:5], raspID)

						whoFor := crwt.ID // or should this be piID??
						copy(messageOut[5:7], whoFor)

						messageType := messageTypeAck // duino message
						messageOut[7] = uint8(messageType)
						sendMessage := rune('1')
						messageOut[8] = uint8(sendMessage)

						// _, err := bm.Conn.WriteToUDP(messageOut, bcast)

						// if err != nil {
						// 	log.Errorf("failed to send ack on BM: %s", err)
						// 	return err
						// }

					}
				}
			}

			// now we update our list of active pis on the network:
			now := time.Now()

			crwt, ok := allPIs[senderID]
			if !ok {
				log.Infof("new PI detected: %+v", senderID)
			}
			crwt.lastMessageReceived = now
			crwt.ID = senderID

			// now do some general house-keeping, set device IDs on the network etc:

			if messageType == messageTypeButton {
				buttonStatus := message[8]
				crwt.ButtonStatus = int64(buttonStatus)
				// log.Infof("buttonStatus: %v", buttonStatus) // 0 is button down, 1 is up
			}

			allPIs[senderID] = crwt

			// remove any PIs we haven't heard from in a while
			for k, v := range allPIs {
				if v.lastMessageReceived.Add(piTTL).Before(now) {
					log.Infof("deleting expired pi: %+v", v)
					delete(allPIs, k)
				}
			}

			log.Infof("current PIs: %d", len(allPIs))
			for _, v := range allPIs {
				log.Infof("  %s", v)
			}

		}

	}
}

func broadcastLoop(keys <-chan rune, raspID string, bcastIP net.IP, bm *readBATMAN.ReadBATMAN) error {
	log.Info("Starting broadcast loop")

	// this is for local messages from locl device hw, key-presses, GPIO press, eventually letter-gesture from TF

	bcast := &net.UDPAddr{Port: batPort, IP: bcastIP}

	crwt, _ := allPIs[raspID]
	crwt.ShieldTimer = time.Now() // reset shield timer

	// // send a quick hello to the BATMAN:
	buttonmsgSize := 9                         // 32(?) bytes for  a hello message
	bMessageOut := make([]byte, buttonmsgSize) // sent to batman
	copy(bMessageOut[0:2], magicByte)
	bMessageOut[2] = uint8(buttonmsgSize)
	copy(bMessageOut[3:5], raspID)

	whoFor := raspiIDEveryone // message for everyone
	//whoFor := OtherID // for the other ID
	copy(bMessageOut[5:7], whoFor)
	log.Infof("whoFor: %s", whoFor)
	messageType := messageTypeHelloA // HELLO Announce
	bMessageOut[7] = uint8(messageType)

	bMessageOut[8] = uint8(5)
	_, err := bm.Conn.WriteToUDP(bMessageOut, bcast)
	if err != nil {
		log.Error(err)
		return err
	}

	for {
		select {

		case key, more := <-keys:
			// received local key press
			// todo: replace/ augment this with a GPIO button press

			if !more {
				log.Infof("keyboard listener closed\n")
				// termbox closed, block until ctrl-c is called
				log.Infof("exiting")
				return nil
			}
			log.Infof("key pressed: %s / %d / 0x%X / 0%o \n", string(key), key, key, key)

			// duino message (9 bytes)
			// 2 bytes: <2 magic bytes>
			// 1 byte:  <total message length, bytes>
			// 2 bytes: <sender ID = 2 bytes, (IP?)>
			// 2 bytes: <who For = 2 bytes, (0= everyone, or ID of)>
			// 1 byte:  <message type (0=gps, 1=duino command, 2=gesture type)>
			// 1 byte:  key

			if (string(key) == "s") || (string(key) == "S") {
				// pressed a shield
				log.Infof("shield pressed!\n")

				now := time.Now()
				// elapsedTime := now.Sub(crwt.ShieldTimer)
				// log.Infof("elapsedTime: %v\n", elapsedTime)

				crwt.ShieldTimer = now // reset shield timer:
				// crwt.ChatRequest.PointDir := 1.0 // reset shield timer: problem here!!
			}

			duinoMsgSize := 9                        // 23 bytes for a duino message
			messageOut := make([]byte, duinoMsgSize) // sent to batman

			copy(messageOut[0:2], magicByte)
			messageOut[2] = uint8(duinoMsgSize)
			copy(messageOut[3:5], raspID)

			whoFor := raspiIDEveryone // everyone

			copy(messageOut[5:7], whoFor)

			//messageType := messageTypeDuino // duino message
			messageType := messageTypeKey // key message
			messageOut[7] = uint8(messageType)

			messageOut[8] = uint8(key)

			_, err := bm.Conn.WriteToUDP(messageOut, bcast)

			if err != nil {
				log.Error(err)
				return err
			}

			// NB now we send message ot duino after we have received it on the net- this way we sync with other duinos better
			// // write to duino: NB maybe insert a wait before here so all pi's send the new duino command at a similar time
			// _, err = duino.Write([]byte(string(key)))
			// if err != nil {
			// 	log.Errorf("2. failed to write to serial port: %s", err)
			// 	return err
			// }

		}
	}
}

// func sayHello() xx {
// 	// send a quick hello to the BATMAN:
// 	buttonmsgSize := 9                         // 32(?) bytes for  a hello message
// 	bMessageOut := make([]byte, buttonmsgSize) // sent to batman
// 	copy(bMessageOut[0:2], magicByte)
// 	bMessageOut[2] = uint8(buttonmsgSize)
// 	copy(bMessageOut[3:5], raspID)

// 	whoFor := raspiIDEveryone // message for everyone
// 	//whoFor := OtherID // for the other ID
// 	copy(bMessageOut[5:7], whoFor)
// 	log.Infof("whoFor: %s", whoFor)
// 	messageType := messageTypeHello // HELLO
// 	bMessageOut[7] = uint8(messageType)

// 	bMessageOut[8] = uint8(5)
// 	_, err := bm.Conn.WriteToUDP(bMessageOut, bcast)
// 	if err != nil {
// 		log.Error(err)
// 		return err
// 	}

// }
