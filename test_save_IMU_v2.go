// test_save_IMU_v2.go

// save IMU outputs to IMAGE file when we press button
// specify pathname to save raw image and quaternions here:
// go run test_save_IMU_v2.go -no-sound -save-dir stewartLee

// connectd with a push button on GPIO and IMU (Bosch BNo055)
// determine what letter the user draws in the air

// NB binary must be run as sudo
// eg go build test_save_IMU_v2.go && sudo ./test_save_IMU_v2 -no-sound

// read switch input from raspberry pi 3+ GPIO and light LED
// when button is down for a "long" time (>500 ms): record IMU data.
// on button-up, we convert the quaternion data from IMU
// (ie accelerometer and gyroscope) into a 28x28 image

package main

import (
	"bufio"
	"encoding/base64"
	"flag"
	"fmt"
	"image"
	"io"
	"os/exec"
	"os/signal"
	"runtime"
	"sort"
	"strconv"
	"strings"
	"syscall"

	"math"
	"os"
	"time"

	"github.com/johnusher/priWand/pkg/acc"
	"github.com/johnusher/priWand/pkg/gpio"
	"github.com/johnusher/priWand/pkg/keyboard"
	"github.com/johnusher/priWand/pkg/oled"

	log "github.com/sirupsen/logrus"
	"golang.org/x/exp/io/i2c"
	"golang.org/x/image/bmp"

	"image/color"
	_ "image/png"
)

const (
	circBufferL = 1200 // length of buffer where we store quat data. 600 samples @5 ms update = 3 seconds
	lp          = 28   // pixels used to represent drawn letter, on each axis, ie lpxlp
)

func main() {

	// parse inut flags for no hardware
	// NB no-sound just means do not output sound- still need I2S connections (probably)
	noACC := flag.Bool("no-imu", false, "run without Bosch IMU")
	noOLED := flag.Bool("no-oled", false, "run without oled display")
	noSound := flag.Bool("no-sound", false, "run without sound")

	logLevel := flag.String("log-level", "info", "log level, must be one of: panic, fatal, error, warn, info, debug, trace")

	saveDir := flag.String("save-dir", "defaultSD", "save dir")

	flag.Parse()

	level, err := log.ParseLevel(*logLevel)
	if err != nil {
		log.Errorf("failed to parse log level [%s]: %s", *logLevel, err)
		return
	}
	log.SetLevel(level)

	// Setup keyboard input:
	stop := make(chan os.Signal, 1)
	signal.Notify(stop, os.Interrupt, syscall.SIGTERM)

	keys := make(chan rune)

	kb, err := keyboard.Init(keys)
	if err != nil {
		log.Errorf("failed to initialize keyboard: %s", err)
		return
	}

	// init TF/ Python

	cmd := exec.Command("python3", "-u", "classifier/classify.py") // linux
	if runtime.GOOS == "windows" {
		cmd = exec.Command("python", "-u", "classifier/classify.py") // windoze
	}

	stdout, err := cmd.StdoutPipe()
	if err != nil {
		log.Errorf("failed to initialize StdoutPipe: %s", err)
		return
	}

	stdin, err := cmd.StdinPipe()
	if err != nil {
		log.Errorf("failed to initialize StdinPipe: %s", err)
		return
	}

	stdoutReader := bufio.NewReader(stdout)

	err = cmd.Start()
	if err != nil {
		log.Errorf("failed to initialize NewReader: %s", err)
		return
	}

	// init accelerometer module (Bosch)
	accChan := make(chan acc.ACCMessage)
	// accChan2 := make(chan acc.ACCMessage2)
	// imu, err := acc.Init(accChan, accChan2, *noACC)
	imu, err := acc.Init(accChan, *noACC)
	if err != nil {
		log.Errorf("failed to initialize acc: %s", err)
		return
	}

	// err = imu.ResetAcc() // bno055OprMode is IMUPLUS = 1000 =0x8
	// if err != nil {
	// 	log.Errorf("failed to change mode: %s", err)
	// }

	// init gpio module:
	gpioChan := make(chan gpio.GPIOMessage)
	// gpio, err := gpio.Init(gpioChan, *noGPIO)  // TBD
	gpio, err := gpio.Init(gpioChan, *noSound)
	if err != nil {
		log.Errorf("failed to initialize GPIO: %s", err)
		return
	}
	defer gpio.Close()

	// OLED:

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

	// make dirs:

	// saveDir := "defaultSaveDir"

	// if !isFlagPassed("saveDir") {
	// 	fmt.Println("saveDir is not passed !!!")
	// 	*saveDir = "defaultSaveDir"
	// }

	log.Printf("saveDirIn: %v", saveDir)

	os.Chdir("letters/")

	if _, err := os.Stat(*saveDir); os.IsNotExist(err) {
		os.Mkdir(*saveDir, 0700)
	}

	os.Chdir(*saveDir)

	OG_dir2, err := os.Getwd()
	if err != nil {
		log.Infof("problem changing dir")
	}

	log.Printf("OG_dir2: %v", OG_dir2)

	// main loop here:

	// go forth
	go kb.Run()
	go imu.Run()

	errs := make(chan error)

	// clear the OLED
	if err := oled.Clear(); err != nil {
		panic(err)
	}

	img := image.NewRGBA(image.Rect(0, 0, 128, 64))

	go func() {
		errs <- GPIOLoop(keys, gpioChan, accChan, img, oled, stdin, stdoutReader, imu)
	}()

	// block until ctrl-c or one of the loops returns an error
	select {
	case <-errs:
	}

}

func GPIOLoop(keys <-chan rune, gpioCh <-chan gpio.GPIOMessage, accCh <-chan acc.ACCMessage, img *image.RGBA, oled oled.OLED, stdin io.WriteCloser, stdoutReader *bufio.Reader, imu acc.ACC) error {
	// log.Info("Starting GPIO loop")

	gpioMessage := gpio.GPIOMessage{}
	accMessage := acc.ACCMessage{}

	buttonDown := false
	n := 0
	var quat_in_circ_buffer [circBufferL][4]float64 // raw quaternion inputs from IMU
	// var gravity_in_circ_buffer [circBufferL][5]float64 // raw gravity inputs from  IMU
	// var euler_in_circ_buffer [circBufferL][5]float64   // raw euler inputs from  IMU

	more := false
	for {

		select {

		case _, more := <-keys:
			// received local key press
			// todo: replace/ augment this with a GPIO button press

			if !more {
				// oled.ShowText(img, 2, fmt.Sprintf("exiting"))
				log.Infof("keyboard listener closed\n")
				// termbox closed, block until ctrl-c is called
				log.Infof("exiting")
				return nil
			}

		case accMessage, more = <-accCh:

			// received message from BNo055 module.
			// eg bearing, ie NSEW direction we are pointing
			if !more {
				log.Infof("acc channel2 closed\n")
				log.Infof("exiting")
				return nil
			}

			if buttonDown {
				// log.Infof("recording quats")
				// log.Infof("n %v", n)

				// todo!!
				// err = acc.Einit()
				// if err != nil {
				// 	return nil, err
				// }

				n = n + 1
				quat_in_circ_buffer[n][0] = accMessage.QuatW
				quat_in_circ_buffer[n][1] = accMessage.QuatX
				quat_in_circ_buffer[n][2] = accMessage.QuatY
				quat_in_circ_buffer[n][3] = accMessage.QuatZ

				// gravity_in_circ_buffer[n][0] = accMessage.GravX
				// gravity_in_circ_buffer[n][1] = accMessage.GravY
				// gravity_in_circ_buffer[n][2] = accMessage.GravZ

				// euler_in_circ_buffer[n][0] = accMessage.Bearing
				// euler_in_circ_buffer[n][1] = accMessage.Roll
				// euler_in_circ_buffer[n][2] = accMessage.Tilt

			}

		case gpioMessage, more = <-gpioCh:

			if !more {
				log.Infof("gpio channel closed\n")
				log.Infof("exiting")
				return nil
			}

			// log.Infof("gpio message %v", gpioMessage)
			// receive a button change from gpio

			buttonStatus := gpioMessage.ButtonFlag
			// buttonStatus := gpio.GPIOMessage.buttonFlag
			if buttonStatus == 0 {
				// button down
				// log.Infof("button down %v", buttonStatus)
				buttonDown = true
				n = 0

				log.Infof("button down")
				// start recording quaternions from IMU

				// err := imu.ResetAcc() // bno055OprMode is IMUPLUS = 1000 =0x8
				// if err != nil {
				// 	log.Errorf("failed to change mode: %s", err)
				// }

			}

			if buttonStatus == 1 {
				// button up
				// log.Infof("button up %v", buttonStatus)
				buttonDown = false

				// stop recording quaternions from IMU,
				// convert quaternions to 28x28 image
				// pipe to TF, Python

				// log.Printf("quat_in_circ_buffer: %v", quat_in_circ_buffer)
				// log.Printf("n: %v", n)
				// mFnbT9sthKKp22GR

				if n > 20 {

					// save to file:
					log.Infof("saving IMAGES. . .")
					// make folder to save:
					t := time.Now()
					// newDir := fmt.Sprintf("%d-%02d-%02d_%02d-%02d-%02d",
					newDir := fmt.Sprintf("L_%02d-%02d-%02d",
						// t.Year(), t.Month(), t.Day(),

						t.Hour(), t.Minute(), t.Second())

					// fmt.Println("Name:", newDir)
					log.Printf("Name: %v", newDir)
					// ioutil.WriteFile(+name, []byte("Contents"), 0)

					if _, err := os.Stat(newDir); os.IsNotExist(err) {
						os.Mkdir(newDir, 0700)
					}

					OG_dir, err := os.Getwd()
					if err != nil {
						log.Infof("problem changing dir")
					}

					log.Printf("OG_dir: %v", OG_dir)

					os.Chdir(newDir)

					// os.Chdir(OG_dir)
					// convert quats to image:
					encoded, letterImage := quats2Image(quat_in_circ_buffer, n)

					// now save quats
					fquat, err := os.Create("quat_data.txt")
					if err != nil {
						panic(err)
					}
					defer fquat.Close()

					length := n
					startOffset := 10
					length = length - startOffset

					for n = 0; n < length; n++ {
						w := quat_in_circ_buffer[n+startOffset][0]
						x := quat_in_circ_buffer[n+startOffset][1]
						y := quat_in_circ_buffer[n+startOffset][2]
						z := quat_in_circ_buffer[n+startOffset][3]
						sw := strconv.FormatFloat(float64(w), 'f', -1, 32)
						sx := strconv.FormatFloat(float64(x), 'f', -1, 32)
						sy := strconv.FormatFloat(float64(y), 'f', -1, 32)
						sz := strconv.FormatFloat(float64(z), 'f', -1, 32)
						_, err := fquat.WriteString(sw + " " + sx + " " + sy + " " + sz + "\n")
						if err != nil {
							panic(err)
						}
					}

					os.Chdir(OG_dir)

					// send encoded base64 28x28 ti TF:
					_, err = stdin.Write([]byte(encoded))
					if err != nil {
						log.Errorf("stdin.Write() failed: %s", err)
					}

					// write end of line:
					_, err = stdin.Write([]byte("\n"))
					if err != nil {
						log.Errorf("stdin.Write() failed: %s", err)
					}

					s2, err := stdoutReader.ReadString('\n')
					if err != nil {
						log.Printf("Process is finished ..")
					}

					// print first and second place:

					log.Printf("raw message: %v", s2)
					s := strings.FieldsFunc(s2, Split)

					prob, _ := strconv.ParseFloat(s[0], 64)
					// letter := strings.Trim(s[1], "'")
					letter := strings.Replace(s[1], "'", "", -1)

					// s[2] is blank
					prob2, _ := strconv.ParseFloat(s[3], 64)
					letter2 := strings.Replace(s[4], "'", "", -1)

					log.Printf("letter1: %v", letter)
					log.Printf("prob1: %v", prob)

					log.Printf("letter2: %v", letter2)
					log.Printf("prob2: %v", prob2)

					// OLED display:
					msgP := fmt.Sprintf("%s or %s", letter, letter2)
					if prob > 0.9 {
						// high probability: just print first place letter
						msgP = fmt.Sprintf("letter = %s", letter)
					}

					TFimg := image.NewRGBA(image.Rect(0, 0, 128, 64))

					oled.ShowText(TFimg, 1, msgP)
					// var letterImage [lp][lp]byte

					oled.AddGesture(TFimg, letterImage)

				} else {
					log.Printf("shorty")
				}

			}

		}

	}

}

func quats2Image(quat_in_circ_buffer [circBufferL][4]float64, length int) (string, [lp][lp]byte) {
	// var imageOut float64
	// imageOut = quatsIn * 2.0
	// n := 0 // index to write into circ buffer
	yaw := make([]float64, circBufferL)
	// sortedyaw := make([]float64, circBufferL)
	angles_diff := make([]float64, circBufferL)
	var maxDiff float64
	var pitch [circBufferL]float64
	var letterImage [lp][lp]byte
	var n int
	var tip [3]float64 = [3]float64{0.8, -0.5, -0.4}
	tipNorm := math.Sqrt(tip[0]*tip[0] + tip[1]*tip[1] + tip[2]*tip[2])
	tip[0] = tip[0] / tipNorm
	tip[1] = tip[1] / tipNorm
	tip[2] = tip[2] / tipNorm

	startOffset := 10
	length = length - startOffset

	// log.Printf(" quat_in_circ_buffer : %v", quat_in_circ_buffer[1:length][:])

	for n = 0; n < length; n++ {
		s := quat_in_circ_buffer[n+startOffset][0]
		x := quat_in_circ_buffer[n+startOffset][1]
		y := quat_in_circ_buffer[n+startOffset][2]
		z := quat_in_circ_buffer[n+startOffset][3]

		x0 := tip[0]
		y0 := tip[1]
		z0 := tip[2]

		w1 := s
		x1 := -x
		y1 := -y
		z1 := -z

		firstMulQuatOut := []float64{-x0*x1 - y0*y1 - z0*z1,
			x0*w1 + y0*z1 - z0*y1,
			-x0*z1 + y0*w1 + z0*x1,
			x0*y1 - y0*x1 + z0*w1}

		w0 := s
		x0 = x
		y0 = y
		z0 = z

		w1 = firstMulQuatOut[0]
		x1 = firstMulQuatOut[1]
		y1 = firstMulQuatOut[2]
		z1 = firstMulQuatOut[3]

		secondMulQuatOut := []float64{w0*w1 - x0*x1 - y0*y1 - z0*z1,
			w0*x1 + x0*w1 + y0*z1 - z0*y1,
			w0*y1 - x0*z1 + y0*w1 + z0*x1,
			w0*z1 + x0*y1 - y0*x1 + z0*w1}

		yaw[n] = math.Atan2(secondMulQuatOut[1], secondMulQuatOut[2]) // add negative for some reason....
		pitch[n] = math.Asin(secondMulQuatOut[3])

	}

	sortedyaw := make([]float64, n)
	copy(sortedyaw, yaw[0:n])
	sort.Float64s(sortedyaw)

	first_angleInd := 0
	angles_diff[0] = (2 * math.Pi) + sortedyaw[0] - sortedyaw[n-1]

	maxDiff = angles_diff[0]
	for i := 1; i < n; i++ {
		angles_diff[i] = sortedyaw[i] - sortedyaw[i-1]
		if angles_diff[i] > maxDiff {
			maxDiff = angles_diff[i]
			first_angleInd = i
		}
	}

	first_angle := sortedyaw[first_angleInd]
	pitchMin := 20.0
	for i := 0; i < n; i++ {
		pitchMin = math.Min(pitchMin, pitch[i])
	}

	for i := 0; i < n-0; i++ {
		pitch[i] = pitch[i] - pitchMin
		yaw[i] = yaw[i] - first_angle // yaw = yaw -  first_angle+0.2;  % yaw -= yaw_range
		if yaw[i] < 0 {
			yaw[i] = yaw[i] + 2*math.Pi
		}
		if pitch[i] < 0 {
			pitch[i] = pitch[i] + 2*math.Pi
		}
	}

	min_yaw := 50.0
	max_yaw := -50.0
	min_pitch := 50.0
	max_pitch := -50.0
	for i := 0; i < n; i++ {
		min_yaw = math.Min(min_yaw, yaw[i])
		max_yaw = math.Max(max_yaw, yaw[i])
		min_pitch = math.Min(min_pitch, pitch[i])
		max_pitch = math.Max(max_pitch, pitch[i])
	}

	x_range := max_yaw - min_yaw
	y_range := max_pitch - min_pitch
	scale := 0.95 / math.Max(x_range, y_range)

	for i := 0; i < n; i++ {
		yaw[i] = 0.5 + (yaw[i]-min_yaw-x_range/2)*scale
		pitch[i] = 0.5 + (pitch[i]-min_pitch-y_range/2)*scale
	}

	// make black and white image:

	// first blank the image:
	for ix := 0; ix < lp; ix++ {
		for iy := 0; iy < lp; iy++ {
			letterImage[ix][iy] = 0
		}
	}

	x_int := 0
	y_int := 0
	for i := 0; i < n; i++ {
		x_int = int(yaw[i] * lp)
		y_int = lp - int(pitch[i]*lp) - 1
		// letterImage[y_int][x_int] = 1
		letterImage[x_int][y_int] = 1
	}

	var joinedArray []byte

	// resize matrix into long array:
	for ix := 0; ix < lp; ix++ {
		nums := letterImage[ix][:]
		joinedArray = append(joinedArray, nums...)
		// joinedArray := bytes.Join(nums, nil)  // dunno how to do this
	}

	encoded := base64.StdEncoding.EncodeToString(joinedArray)

	// // -------------------------------
	// // Create png image
	img := image.NewRGBA(image.Rect(0, 0, lp, lp))

	// x_int = 0
	// y_int = 0
	for i := 0; i < n; i++ {
		// x_int = lp - int(yaw[i]*lp)
		// y_int = lp - int(pitch[i]*lp)

		x_int = int(yaw[i] * lp)
		y_int = lp - int(pitch[i]*lp) - 1

		img.Set(x_int, y_int, color.RGBA{255, 0, 0, 255})
	}

	// sn := strings.Replace(file, "quaternion_data.txt", "quat_image.bmp", 1)
	// sn := strings.Replace(file, "quat_data.txt", "quat_image.bmp", 1)

	sn := "imageFile.bmp"
	fo, err := os.OpenFile(sn, os.O_WRONLY|os.O_CREATE, 0600)
	if err != nil {
		log.Printf("err %s\n", err)
	}

	defer fo.Close()
	bmp.Encode(fo, img)

	return encoded, letterImage
}

func GetFilenameDate() string {
	// Use layout string for time format.
	// const layout = "01-02-2006"
	const layout = "15:04:05"

	// Place now in the string.
	t := time.Now()
	return "file-" + t.Format(layout) + ".bmp"
}

// sudo chmod 777 *.bmp

func Split(r rune) bool {
	return r == ':' || r == ',' || r == '(' || r == ')' || r == '[' || r == ']'
}

func isFlagPassed(name string) bool {
	found := false
	flag.Visit(func(f *flag.Flag) {
		if f.Name == name {
			found = true
		}
	})
	return found
}
