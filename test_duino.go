// test-duino.go
// send serial commands to duino (actually, use ATmega328P )
// use usb connection (/dev/ttyUSB0 etc)
// for code running on the duino see duino_src/duino_src:
// serial messages should be semi-colon delimited
// eg 0;20;40;10
// first value is mode
// mode 0 = solid [r g b]
// mode 1 = flash [r g b onTime offTime]
// mode 2 = random [not quite working]
// mode 3 = rainbow fade,  second argument should be 25 for fast fade
// mode 9 = set brightness (right shift amount of GRB)

package main

import (
	"flag"
	"fmt"
	"io/ioutil"

	// "math/rand"

	"strings"
	"time"

	"github.com/johnusher/ardpifi/pkg/port"
	log "github.com/sirupsen/logrus"
	"github.com/tarm/serial"

	_ "image/png"
	// "github.com/goiot/devices/monochromeoled"
)

const ()

func main() {

	noDuino := flag.Bool("no-duino", false, "run without arduino")
	logLevel := flag.String("log-level", "info", "log level, must be one of: panic, fatal, error, warn, info, debug, trace")

	flag.Parse()

	level, err := log.ParseLevel(*logLevel)
	if err != nil {
		log.Errorf("failed to parse log level [%s]: %s", *logLevel, err)
		return
	}
	log.SetLevel(level)

	// Find the device that represents the arduino serial
	// connection. NB this is kinda janky- we should have a system to robustly detect a duino,
	// eg if we dont find one, then re-insert the duino USb cable and note which ports are new

	c := &serial.Config{Name: findArduino(), Baud: 115200, ReadTimeout: time.Second * 1}

	// c := &serial.Config{Name: "/dev/ttyUSB0", Baud: 115200, ReadTimeout: time.Second * 1}

	duino, err := port.OpenPort(c, *noDuino)
	if err != nil {
		log.Errorf("OpenPort error: %s", err)
		return
	}

	// When connecting to an older revision Arduino, you need to wait
	time.Sleep(1 * time.Second)

	// var message []byte

	// duinoMessage := "0;255;255;64\n"   ///c change to solid colour

	// duinoMessage := "1;255;255;64;350;250\n" // mode 1 = flash solid colour, ontime = 350, offtime = 250

	duinoMessage := "3;25\n" // mode 3 = rainbow fade, speed=25

	// duinoMessage := "9;4\n" // increase brightness [mdoe 9, codeload = right shift]

	// write to duino:
	duino.Flush()
	_, err = duino.Write([]byte(duinoMessage))

	if err != nil {
		log.Errorf("3. failed to write to duino: %s", err)
		//return err
	}
	duino.Flush()

	log.Infof("duinoMessage %s \n", (string(duinoMessage)))

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
