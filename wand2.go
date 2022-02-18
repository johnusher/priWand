// wand2.go
// aka pea-shooter

// GPIO button press

// requires 2 wands, outside with good GPS lock.
// point at another wand, press button to zap the other wand.

// combines JU_led_mesh.go and test_GPIO2.go

// key press Q to quit

// pi4: go run wand2.go -rasp-id=67 --web-addr :8082 -no-batman -log-level debug
// go run wand2.go -rasp-id=68 --web-addr :8082 -no-oled -no-duino -no-acc -no-sound -log-level debug

// push from 4->3:
// rsync -a wand2.go pi@192.168.1.166:code/go/src/github.com/johnusher/priWand/

package main

import (
	"encoding/binary"
	"flag"
	"fmt"
	"io/ioutil"
	"math"

	// "math/rand"
	"net"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	// "github.com/qinxin0720/lcd1602"

	// i2c "github.com/d2r2/go-i2c"

	// "github.com/johnusher/priWand/pkg/gpio"

	"github.com/johnusher/priWand/pkg/acc"
	"github.com/johnusher/priWand/pkg/gpio"
	"github.com/johnusher/priWand/pkg/gps"
	"github.com/johnusher/priWand/pkg/iface"
	"github.com/johnusher/priWand/pkg/keyboard"

	// "github.com/johnusher/priWand/pkg/lcd"
	"github.com/johnusher/priWand/pkg/oled"
	"github.com/johnusher/priWand/pkg/port"
	"github.com/johnusher/priWand/pkg/readBATMAN"
	"github.com/johnusher/priWand/pkg/web"

	log "github.com/sirupsen/logrus"
	"github.com/tarm/serial"

	"image"

	_ "image/png"

	// "github.com/goiot/devices/monochromeoled"
	"golang.org/x/exp/io/i2c"
)

const (
	bearingThreshold = 40 // value in degrees. if we are within eg 40 degrees pointing at another, then consider it a lock on
	batPort          = 4200
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
	messageTypeButton = 3

	raspiIDEveryone = "00"
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
}

type chatRequestWithTimestamp struct {
	ChatRequest
	lastMessageReceived time.Time
}

// String satisfies the Stringer interface
func (c ChatRequest) String() string {
	return fmt.Sprintf("id: %s, coords: (%f, %f), HDOP: %.2f", c.ID, c.Latf, c.Longf, c.HDOPf)
}

// String satisfies the Stringer interface
func (c chatRequestWithTimestamp) String() string {
	return fmt.Sprintf("%s, age: %s]", c.ChatRequest, time.Now().Sub(c.lastMessageReceived))
}

func main() {
	raspID := flag.String("rasp-id", "60", "unique raspberry pi ID") // only 2 characters. use last 2 digits of IP
	webAddr := flag.String("web-addr", ":8080", "address to serve web on")
	noBatman := flag.Bool("no-batman", false, "run without batman network")
	noDuino := flag.Bool("no-duino", false, "run without arduino")
	noGPS := flag.Bool("no-gps", false, "run without gps")
	noOLED := flag.Bool("no-oled", false, "run without oled display")
	noACC := flag.Bool("no-acc", false, "run without Bosch accelerometer")
	noSound := flag.Bool("no-sound", false, "run without sound") // NB no-sound just means do not output sound- still need I2S connections (probably)

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

	// OLED:

	// TODO: put all of this into a function called "setup OLED"

	oled, err := oled.Open(&i2c.Devfs{Dev: "/dev/i2c-1"}, *noOLED)
	if err != nil {
		panic(err)
	}
	defer oled.Close()

	// load png and display on OLED
	rc, err := os.Open("./maxi.png")

	if err != nil {
		panic(err)
	}
	defer rc.Close()

	m, _, err := image.Decode(rc)
	if err != nil {
		panic(err)
	}

	// clear the display before putting on anything
	if err := oled.Clear(); err != nil {
		panic(err)
	}

	if err := oled.SetImage(0, 0, m); err != nil {
		panic(err)
	}
	if err := oled.Draw(); err != nil {
		panic(err)
	}

	// End of OLED

	// now start web broadcast

	web := web.InitWeb(*webAddr)
	log.Infof("web: %+v", web)

	bcastIP := net.ParseIP(batBcast)
	if *noBatman {
		bcastIP = net.ParseIP(localBcast)
	}

	// Find the device that represents the arduino serial
	// connection. NB this is kinda janky- we should have a system to robustly detect a duino,
	// eg if we dont find one, then re-insert the duino USb cable and note which ports are new

	// c := &serial.Config{Name: findArduino(), Baud: 9600, ReadTimeout: time.Second * 1}
	// c := &serial.Config{Name: findArduino(), Baud: 19200, ReadTimeout: time.Second * 1}
	c := &serial.Config{Name: findArduino(), Baud: 115200, ReadTimeout: time.Second * 1}

	duino, err := port.OpenPort(c, *noDuino)
	if err != nil {
		log.Errorf("OpenPort error: %s", err)
		return
	}

	// When connecting to an older revision Arduino, you need to wait
	time.Sleep(1 * time.Second)

	//duinoMessage := "3;25\n" // mode 3 = rainbow fade, speed=25

	duinoMessage := "9;2\n" // increase brightness [mdoe 9, codeload = right shift]
	// write to duino:
	duino.Flush()
	_, err = duino.Write([]byte(duinoMessage))
	if err != nil {
		log.Errorf("3. failed to write to serial port: %s", err)
		//return err
	}
	duino.Flush()
	duinoMessage = "3;5\n" // mode 3 = rainbow fade, speed=25
	// duinoMessage = "1;255;16;240;100;50\n" // mode 1 = flash solid colour, RGB, ontime = 350, offtime = 250
	// duinoMessage := "9;4\n" // increase brightness [mdoe 9, codeload = right shift]
	// write to duino:
	duino.Flush()
	_, err = duino.Write([]byte(duinoMessage))
	if err != nil {
		log.Errorf("3. failed to write to serial port: %s", err)
		//return err
	}
	duino.Flush()

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

	// init accelerometer module (Bosch)
	accChan := make(chan acc.ACCMessage)
	a, err := acc.Init(accChan, *noACC)
	if err != nil {
		log.Errorf("failed to initialize acc: %s", err)
		return
	}
	// defer a.Close()  // why do we comment this?

	// init GPS module:
	gpsChan := make(chan gps.GPSMessage)
	g, err := gps.Init(gpsChan, *noGPS)
	if err != nil {
		log.Errorf("failed to initialize gps: %s", err)
		return
	}
	defer g.Close()

	// init gpio module:
	gpioChan := make(chan gpio.GPIOMessage)
	// gp, err := gpio.Init(gpioChan, *noGPIO)  // TBD
	gp, err := gpio.Init(gpioChan, *noSound)
	if err != nil {
		log.Errorf("failed to initialize GPIO: %s", err)
		return
	}
	defer gp.Close()

	// go forth
	go kb.Run()
	go bm.Run()
	go g.Run()
	go a.Run()

	// show slow rainbow as default when no button pressed
	duinoMessage = "3;125\n" // mode 3 = rainbow fade, speed=25
	duino.Flush()
	_, err = duino.Write([]byte(duinoMessage))
	if err != nil {
		log.Errorf("3. failed to write to serial port: %s", err)
	}
	duino.Flush()

	gp.PlayWav("hello.wav") // play wav

	// log.Info("gp, %v", gp)

	errs := make(chan error)

	// clear the OLED
	if err := oled.Clear(); err != nil {
		panic(err)
	}
	img := image.NewRGBA(image.Rect(0, 0, 128, 64))

	// now we run the two loops
	// receiveBATMAN: listen for incoming, eg button press or messages on the BATMAN
	// broadcastLoop: send message to BATMAN

	go func() {
		// errs <- receiveBATMAN(messages, accChan, duino, *raspID, img, oled, web, bcastIP, bm)
		errs <- receiveBATMAN(messages, accChan, duino, *raspID, img, oled, web, bcastIP, bm, gp)
	}()
	go func() {
		errs <- broadcastLoop(keys, gpsChan, duino, *raspID, bcastIP, bm, img, oled, gpioChan)
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
// func receiveBATMAN(messages <-chan []byte, accCh <-chan acc.ACCMessage, duino port.Port, raspID string, img *image.RGBA, oled oled.OLED, web *web.Web, bcastIP net.IP, bm *readBATMAN.ReadBATMAN) error {
func receiveBATMAN(messages <-chan []byte, accCh <-chan acc.ACCMessage, duino port.Port, raspID string, img *image.RGBA, oled oled.OLED, web *web.Web, bcastIP net.IP, bm *readBATMAN.ReadBATMAN, gp gpio.GPIO) error {

	log.Info("Starting message loop")
	// listen for new incoming BATMAN message
	// allPIs keeps track of the last message received from each PI, keyed by
	// raspID
	allPIs := map[string]chatRequestWithTimestamp{}
	accMessage := acc.ACCMessage{}
	bcast := &net.UDPAddr{Port: batPort, IP: bcastIP}

	crwt, _ := allPIs[raspID]
	crwt.ButtonStatus = 0 // init button status to button up: for some reason this should be a 1 for up but ...

	more := false

	for {

		select {

		case accMessage, more = <-accCh:

			// received message from BNo055 module.
			// eg bearing, ie NSEW direction we are pointing
			if !more {
				log.Infof("receiveBATMAN closing\n")
				log.Infof("exiting")
				return nil
			}

			// accMessage.Bearing is "pointing direction" of self
			// note we also use the term "Bearing" for the relative direction between pis
			// from the mock, accMessage.Bearing was always rand * 0.0001, so never
			// larger than 0.0001, so always rounded to zero
			bearingI := int64(math.Round(accMessage.Bearing))

			// save to self
			crwt, _ := allPIs[raspID]
			crwt.PointDir = bearingI // trying to save here but later, it just pulls a zero!
			allPIs[raspID] = crwt

			// msgP := fmt.Sprintf("Pointing direction = %d", bearingI)
			// log.Infof(msgP)

			// OLED display:
			msgP := fmt.Sprintf("Pointing = %d", bearingI)
			oled.ShowText(img, 1, msgP)

		case message, _ := <-messages:
			// message on the BATMAN

			magicBytesRx := string(message[0:2]) // combine the magicBytes

			if magicBytesRx != magicByte {
				log.Errorf("Received magicBytes %s, expected %s", string(magicBytesRx), magicByte)
				continue
			}

			// messageLength := uint8(message[2])  // todo: check message is correct length!

			senderID := string(message[3:5]) // this is length of raspID = 2 bytes

			whoFor := string(message[5:7]) // whoFor is also length of raspID = 2 bytes

			messageType := message[7]

			if senderID == raspID {
				// // senderID and raspID should both be two bytes, ie two characters
				// if messageType == messageTypeDuino {
				// 	// write to duino:
				// 	// this is currently kinda redundant, ie whether the message is from self or other, we send it to duino
				// 	// ... but one day we may send a different message for self-sent message

				// 	// first unpack the message:
				// 	duinoMessage := message[8] // we should maybe look at total message legnth and combine other bytes if longer than 7

				// 	// write to duino:
				// 	duino.Flush()
				// 	_, err := duino.Write([]byte(string(duinoMessage)))

				// 	if err != nil {
				// 		log.Errorf("3. failed to write to serial port: %s", err)
				// 		//return err
				// 	}
				// 	duino.Flush()
				// }

			} else {

				if whoFor == raspiIDEveryone || whoFor == raspID { // the strcmp with whoFor doesnt work!!
					// if message[6] == 0 || whoFor == raspID { // message[6] == 0  means for everyone.
					// message is not sent by self and is for everyone or for me

					if messageType == messageTypeDuino {

						// duino command: send straight to duino

						// duinoMessage := "0;255;255;64\n" ///c change to solid colour
						// duinoMessage = "3;125\n" // mode 3 = rainbow fade, speed=25
						duinoMessage := "1;255;0;0;200;75\n" // mode 1 = flash solid colour, RGB, ontime = 350, offtime = 250
						// duinoMessage := "9;4\n" // increase brightness [mdoe 9, codeload = right shift]
						// write to duino:
						duino.Flush()
						_, err := duino.Write([]byte(duinoMessage))
						if err != nil {
							log.Errorf("3. failed to write to serial port: %s", err)
						}
						duino.Flush()

						// first unpack the message:
						duinoMessagek := message[8] // we should maybe look at total message legnth and combine other bytes if longer than 7

						log.Infof("key from other %s \n", (string(duinoMessagek)))

						// OLED display:
						OLEDmsg := fmt.Sprintf("received:  %s", (string(duinoMessagek)))
						oled.ShowText(img, 2, OLEDmsg)

						// now we want to play wav
						// wav play is handled in pkg gpio
						gp.PlayWav("hello.wav") // play wav
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

			if messageType == messageTypeGPS {
				// gps package: work out relative location of self to this other wand

				// Received Lattitude is a float 64 in message bytes 8:15
				// Received Long is a float 64 in message bytes 16:23

				rxLatBytes := message[8:16]
				bits := binary.LittleEndian.Uint64(rxLatBytes)
				rxLatFloat := math.Float64frombits(bits)

				crwt.Latf = rxLatFloat

				rxLongBytes := message[16:24]
				bits = binary.LittleEndian.Uint64(rxLongBytes)
				rxLongFloat := math.Float64frombits(bits)

				crwt.Longf = rxLongFloat

				rxHDOPBytes := message[24:32]
				bits = binary.LittleEndian.Uint64(rxHDOPBytes)
				rxHDOPFloat := math.Float64frombits(bits)

				crwt.HDOPf = rxHDOPFloat

				// todo: HDOP
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

			if messageType == messageTypeGPS {
				// if messageType == messageTypeButton {

				if self, ok := allPIs[raspID]; ok && len(allPIs) > 1 {
					// we have >1 Pis, including ourself, find bearing and distance from local to each pi

					log.Infof("buttonStatus: %v", self.ButtonStatus) // 0 is button down, 1 is up

					// NB should we also do this when we have a new estimate for our local GPS location?

					lat1 := self.Latf
					long1 := self.Longf

					currentPD := self.PointDir // current pointing direction of self/ this returns zeros!!

					// msgP := fmt.Sprintf("Pointing direction = %d", currentPD)
					// log.Infof(msgP)

					for piID, crwt := range allPIs {
						if piID == raspID {
							// this is ourself, skip
							continue
						}

						lat2 := crwt.Latf
						long2 := crwt.Longf

						bearing, _ := gps.CalcGPSBearing(lat1, long1, lat2, long2)
						distance := gps.CalcGPSdistance(lat1, long1, lat2, long2)
						bearingI := int64(math.Round(bearing))
						distI := int64(math.Round(distance))

						// now see if bearing to this other pi matches pointing direction of the current pi AND current pi has button down
						bearingMistmatch := int64(1)
						if currentPD > 360-bearingThreshold && bearingI < bearingThreshold {
							bearingMistmatch = Abs(currentPD - (bearingI + 360))
						} else if bearingI > 360-bearingThreshold && currentPD < bearingThreshold {
							bearingMistmatch = Abs(bearingI - (currentPD + 360))
						} else {
							bearingMistmatch = Abs(currentPD - bearingI)
						}

						// msgP := fmt.Sprintf("currentPD, %d", currentPD)
						// log.Infof(msgP)

						// msgP = fmt.Sprintf("bearingMistmatch, %d", bearingMistmatch)
						// log.Infof(msgP)

						if bearingMistmatch < bearingThreshold && self.ButtonStatus == 1 {
							msgP := fmt.Sprintf("We are pointing at %s, but button is not down", crwt.ID)
							log.Infof(msgP)
						}

						if bearingMistmatch < bearingThreshold && self.ButtonStatus == 0 {
							// we are pointing at another AND we have button down
							// TODO we shuld check both HDOPs are <1.5 ?

							// send key=1 to network, to the piID. ie using broadcastLoop

							// duino message (9 bytes)
							// 2 bytes: <2 magic bytes>
							// 1 byte:  <total message length, bytes>
							// 2 bytes: <sender ID = 2 bytes, (IP?)>
							// 2 bytes: <who For = 2 bytes, (0= everyone, or ID of)>
							// 1 byte:  <message type (0=gps, 1=duino command, 2=gesture type)>
							// 1 byte:  key

							duinoMsgSize := 9                        // 23 bytes for a duino message
							messageOut := make([]byte, duinoMsgSize) // sent to batman

							copy(messageOut[0:2], magicByte)
							messageOut[2] = uint8(duinoMsgSize)
							copy(messageOut[3:5], raspID)

							// send just to the one we are pointing at:
							whoFor := crwt.ID // or should this be piID??
							copy(messageOut[5:7], whoFor)

							messageType := messageTypeDuino // duino message
							messageOut[7] = uint8(messageType)
							sendMessage := rune('1')
							messageOut[8] = uint8(sendMessage)

							_, err := bm.Conn.WriteToUDP(messageOut, bcast)

							if err != nil {
								log.Error(err)
								return err
							}

							msgP := fmt.Sprintf("We are pointing at %s", crwt.ID)
							log.Infof(msgP)

							msgP = fmt.Sprintf("self.ButtonStatus  %v", self.ButtonStatus)
							log.Infof(msgP)

							// msgP = fmt.Sprintf("Pointing at", crwt.ID)
							// oled.ShowText(img, 5, msgP)

						}

						msg1 := fmt.Sprintf("bearing to %s = %d\xB0", crwt.ID, bearingI)
						//log.Infof(msg1)
						msg2 := fmt.Sprintf("dist to %s = %d m", crwt.ID, distI)
						//log.Infof(msg2)

						// msg1 = fmt.Sprintf("bearing = %d\xB0", bearingI)
						msg1 = fmt.Sprintf("bearing = %d", bearingI)
						msg2 = fmt.Sprintf("dist = %d m", distI)
						oled.ShowText(img, 3, msg1)
						oled.ShowText(img, 4, msg2)
					}
				}
			}
		}

	}
}

func broadcastLoop(keys <-chan rune, gpsCh <-chan gps.GPSMessage, duino port.Port, raspID string, bcastIP net.IP, bm *readBATMAN.ReadBATMAN, img *image.RGBA, oled oled.OLED, gpioCh <-chan gpio.GPIOMessage) error {
	log.Info("Starting broadcast loop")

	// this is for local messages from locl device hw, eg key-presses, GPS update, pointing direction, GPIO press

	bcast := &net.UDPAddr{Port: batPort, IP: bcastIP}
	gpsMessage := gps.GPSMessage{}

	// buttonDown := false
	n := 0 // counts how long we have button down
	gpioMessage := gpio.GPIOMessage{}

	more := false

	for {
		select {

		case gpioMessage, more = <-gpioCh:

			if !more {
				log.Infof("gpio channel closed\n")
				log.Infof("exiting")
				return nil
			}

			// log.Infof("gpio message %v", gpioMessage)
			// receive a button change from gpio

			buttonmsgSize := 9                         // 32(?) bytes for  a button message
			bMessageOut := make([]byte, buttonmsgSize) // sent to batman

			copy(bMessageOut[0:2], magicByte)
			bMessageOut[2] = uint8(buttonmsgSize)
			copy(bMessageOut[3:5], raspID)

			whoFor := raspiIDEveryone // message for everyone
			copy(bMessageOut[5:7], whoFor)

			messageType := messageTypeButton // GPIO
			bMessageOut[7] = uint8(messageType)

			buttonStatus := gpioMessage.ButtonFlag

			bMessageOut[8] = uint8(buttonStatus)

			_, err := bm.Conn.WriteToUDP(bMessageOut, bcast)

			if err != nil {
				log.Error(err)
				return err
			}

			// buttonStatus := gpio.GPIOMessage.buttonFlag
			if buttonStatus == 0 {
				// button down
				duinoMessage := "0;176;38;255\n" ///c change to solid colour
				duino.Flush()
				_, err := duino.Write([]byte(duinoMessage))
				if err != nil {
					log.Errorf("failed to write to duino: %s", err)
				}
				duino.Flush()

				// log.Infof("button down %v", buttonStatus)
				// buttonDown = true
				n = 0
				// start recording quaternions from IMU
			}

			if buttonStatus == 1 {
				// button up
				duinoMessage := "3;125\n" // mode 3 = rainbow fade, speed=25
				duino.Flush()
				_, err := duino.Write([]byte(duinoMessage))
				if err != nil {
					log.Errorf("3. failed to write to duino: %s", err)
				}
				duino.Flush()
				// log.Infof("button up %v", buttonStatus)
				// buttonDown = false

				// stop recording quaternions from IMU,
				// convert quaternions to 28x28 image
				// pipe to TF, Python

				if n > 20 {

				} else {
					log.Printf("shorty")
					duinoMessage := "3;125\n" // mode 3 = rainbow fade, speed=25
					duino.Flush()
					_, err := duino.Write([]byte(duinoMessage))
					if err != nil {
						log.Errorf("3. failed to write to duino: %s", err)
					}
					duino.Flush()
				}

			}

		case gpsMessage, more = <-gpsCh:

			// received GPS from local GPS module

			if !more {
				log.Infof("gps channel closed\n")
				log.Infof("exiting")
				return nil
			}

			// GPS message (32 bytes)
			// 2 bytes: <2 magic bytes>
			// 1 byte:  <total message length, bytes>
			// 2 bytes: <sender ID = 2 bytes, (IP?)>
			// 2 bytes: <who For = 2 bytes, (0= everyone, or ID of)>
			// 1 byte:  <message type (0=gps, 1=duino command, 2=gesture type)>
			// 8 bytes: Lat
			// 8 bytes: Long
			// 8 bytes: HDOP

			if gpsMessage.Lat != 0.0 {

				// GPSmsgSize := 24                       // 24 bytes for  a gps message

				GPSmsgSize := 32                       // 32 bytes for  a gps message
				messageOut := make([]byte, GPSmsgSize) // sent to batman

				copy(messageOut[0:2], magicByte)
				messageOut[2] = uint8(GPSmsgSize)
				copy(messageOut[3:5], raspID)

				whoFor := raspiIDEveryone // message for everyone
				copy(messageOut[5:7], whoFor)

				messageType := messageTypeGPS // GPS
				messageOut[7] = uint8(messageType)

				// now split the float64 lat and long values into bytes and shove them in the message
				binary.LittleEndian.PutUint64(messageOut[8:16], math.Float64bits(gpsMessage.Lat))
				binary.LittleEndian.PutUint64(messageOut[16:24], math.Float64bits(gpsMessage.Long))
				binary.LittleEndian.PutUint64(messageOut[24:32], math.Float64bits(gpsMessage.HDOP))

				_, err := bm.Conn.WriteToUDP(messageOut, bcast)
				if err != nil {
					log.Error(err)
					return err
				}

			}

			// OLED display:
			if gpsMessage.HDOP != 0.0 {
				msgP := fmt.Sprintf("HDOP = %.2f", gpsMessage.HDOP)
				oled.ShowText(img, 6, msgP)
			}

		case key, more := <-keys:
			// received local key press
			// todo: replace/ augment this with a GPIO button press

			if !more {
				oled.ShowText(img, 2, fmt.Sprintf("exiting"))
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

			duinoMsgSize := 9                        // 23 bytes for a duino message
			messageOut := make([]byte, duinoMsgSize) // sent to batman

			copy(messageOut[0:2], magicByte)
			messageOut[2] = uint8(duinoMsgSize)
			copy(messageOut[3:5], raspID)

			whoFor := raspiIDEveryone // everyone
			copy(messageOut[5:7], whoFor)

			messageType := messageTypeDuino // duino message
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

			// OLED display:
			oled.ShowText(img, 2, fmt.Sprintf("key pressed: %s", string(key)))

		}
	}
}

// findArduino looks for the file that represents the Arduino
// serial connection.

func findArduino() string {
	contents, _ := ioutil.ReadDir("/dev")

	// Look for what is mostly likely the Arduino device
	// NB this is kinda janky- we should have a system to robustly detect a duino, eg if we dont find one, then re-insert the duino USb cable and note which ports are new

	// on my RASPI the legit Aurdion Uno shows in ttyACM0, but my fake nano +CH340-Chip shows on ttyUSB0
	for _, f := range contents {
		if strings.Contains(f.Name(), "ttyUSB0") || strings.Contains(f.Name(), "ttyUSB") || strings.Contains(f.Name(), "ttyACM0") {
			fmt.Println("Duino found at /dev/", f.Name())
			return "/dev/" + f.Name()
		}
	}

	// Have not been able to find a USB device that 'looks'
	// like an Arduino.
	return ""
}

// Abs returns the absolute value of x.
func Abs(x int64) int64 {
	if x < 0 {
		return -x
	}
	return x
}
