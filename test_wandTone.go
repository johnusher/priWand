// test_wandTone

// movement of IMU changes tone

// save IMU outputs to txt file when we press button

// t
// connectd with a push button on GPIO and IMU (Bosch BNo055)
// determine what letter the user draws in the air

// NB binary must be run as sudo
// eg go build test_record_spell.go && sudo ./test_record_spell -no-sound

// read switch input from raspberry pi 3+ GPIO and light LED
// when button is down for a "long" time (>500 ms): record IMU data.
// on button-up, we convert the quaternion data from IMU (ie accelerometer and gyroscope) into a 28x28 image
// the image is then piped to a tensorflowlite classify model in python
// the python app then returns the best guess letter and %prob

package main

import (
	"bufio"
	"encoding/base64"
	"errors"
	"flag"
	"image"
	"io"
	"os/exec"
	"os/signal"
	"runtime"
	"syscall"

	"math"
	"os"
	"time"

	"github.com/johnusher/priWand/pkg/acc"
	"github.com/johnusher/priWand/pkg/gpio"
	"github.com/johnusher/priWand/pkg/keyboard"
	"github.com/johnusher/priWand/pkg/oled"

	"github.com/faiface/beep"
	"github.com/faiface/beep/speaker"

	log "github.com/sirupsen/logrus"
	"golang.org/x/exp/io/i2c"

	_ "image/png"
)

type SineWave struct {
	sampleFactor float64 // Just for ease of use so that we don't have to calculate every sample
	phase        float64
}

func (g *SineWave) Stream(samples [][2]float64) (n int, ok bool) {
	for i := range samples { // increment = ((2 * PI) / SampleRate) * freq
		v := math.Sin(g.phase * 2.0 * math.Pi) // period of the wave is thus defined as: 2 * PI.
		samples[i][0] = v
		samples[i][1] = v
		_, g.phase = math.Modf(g.phase + g.sampleFactor)
	}

	return len(samples), true
}

func (*SineWave) Err() error {
	return nil
}

func SineTone(sr beep.SampleRate, freq float64) (beep.Streamer, error) {
	dt := freq / float64(sr)

	if dt >= 1.0/2.0 {
		return nil, errors.New("faiface sine tone generator: samplerate must be at least 2 times grater then frequency")
	}

	return &SineWave{dt, 0.1}, nil
}

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

	// main loop here:

	os.Chdir("letters/feb22")

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

	sr := beep.SampleRate(24000)
	speaker.Init(sr, sr.N(time.Second/10)) // sr.N(time.Second/10) = buffer size for duration 1/10 second
	// sine, _ := SineTone(sr, 343)
	// speaker.Play(sine)

	gpioMessage := gpio.GPIOMessage{}
	accMessage := acc.ACCMessage{}

	buttonDown := false
	n := 0
	// var quat_in_circ_buffer [circBufferL][5]float64    // raw quaternion inputs from IMU
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
				// quat_in_circ_buffer[n][0] = accMessage.QuatW
				// quat_in_circ_buffer[n][1] = accMessage.QuatX
				// quat_in_circ_buffer[n][2] = accMessage.QuatY
				// quat_in_circ_buffer[n][3] = accMessage.QuatZ

				// gravity_in_circ_buffer[n][0] = accMessage.GravX
				// gravity_in_circ_buffer[n][1] = accMessage.GravY
				// gravity_in_circ_buffer[n][2] = accMessage.GravZ

				// euler_in_circ_buffer[n][0] = accMessage.Bearing
				// euler_in_circ_buffer[n][1] = accMessage.Roll
				// euler_in_circ_buffer[n][2] = accMessage.Tilt

				// ctrl := &beep.Ctrl{}
				// // ctrl.Paused = true
				// ctrl.Streamer = nil
				// speaker.Play(ctrl)

				if n%15 == 0 && n > 130 {
					// speaker.Clear()
					sine, _ := SineTone(sr, accMessage.Tilt*10)
					speaker.Play(sine)
				}

				// speaker.Lock()

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

				// sine, _ := SineTone(sr, 343)
				// speaker.Play(sine)

			}

			if buttonStatus == 1 {
				// button up
				log.Infof("button up %v", buttonStatus)
				buttonDown = false

				speaker.Clear()

				if n > 20 {

				} else {
					log.Printf("shorty")
				}

			}

		}

	}

}

func quats2Image(quat_in_circ_buffer [circBufferL][5]float64, length int) (string, [lp][lp]byte) {
	// var imageOut float64
	// imageOut = quatsIn * 2.0
	// n := 0 // index to write into circ buffer

	// var s, x, y, z float64
	var projected_circ_buffer [circBufferL][3]float64 // quat projected to xyz
	var letterImage [lp][lp]byte
	var centre [3]float64 // 3x1 averages of projected_circ_buffer coluns
	var n int
	var centre_direction [3]float64 // 3x1
	var centre_direction_sq_centre_col [3]float64
	var eye_minus_cdscc [3]float64
	var y_direction [3]float64
	var x_direction [3]float64
	var x [circBufferL]float64
	var y [circBufferL]float64

	startOffset := 10
	length = length - startOffset

	// for n = 0; n < circBufferL; n++ {
	for n = 0; n < length; n++ {

		s := quat_in_circ_buffer[n+startOffset][0]
		x := quat_in_circ_buffer[n+startOffset][1]
		y := quat_in_circ_buffer[n+startOffset][2]
		z := quat_in_circ_buffer[n+startOffset][3]

		projected_circ_buffer[n][0] = 1.0 - 2.0*(y*y+z*z)
		projected_circ_buffer[n][1] = 2.0 * (x*y + s*z)
		projected_circ_buffer[n][2] = 2.0 * (x*z - s*y)

	}
	// step 3. when we have stopped recording data: average of projected

	sumx := 0.0
	sumz := 0.0
	sumy := 0.0

	for i := 0; i < n; i++ {
		sumx += (projected_circ_buffer[i][0])
		sumy += (projected_circ_buffer[i][1])
		sumz += (projected_circ_buffer[i][2])
	}

	centre[0] = (sumx) / (float64(n))
	centre[1] = (sumy) / (float64(n))
	centre[2] = (sumz) / (float64(n))

	// %% step 4: norm-centre

	// %     centre_direction = centre' ./ norm(centre);

	norm_centre := math.Sqrt(centre[0]*centre[0] + centre[1]*centre[1] + centre[2]*centre[2])

	centre_direction[0] = centre[0] / norm_centre
	centre_direction[1] = centre[1] / norm_centre
	centre_direction[2] = centre[2] / norm_centre

	// % step 5: y direction:
	// %     y_direction = (eye[2] - centre_direction*centre_direction')* [0 1 0]';

	centre_direction_sq_centre_col[0] = centre_direction[0] * centre_direction[1]
	centre_direction_sq_centre_col[1] = centre_direction[1] * centre_direction[1]
	centre_direction_sq_centre_col[2] = centre_direction[1] * centre_direction[2]

	eye_minus_cdscc[0] = -1.0 * centre_direction_sq_centre_col[0]
	eye_minus_cdscc[1] = 1.0 - centre_direction_sq_centre_col[1]
	eye_minus_cdscc[2] = -1.0 * centre_direction_sq_centre_col[2]

	// %     y_direction = y_direction ./ norm(y_direction);

	norm_y_direction := math.Sqrt(eye_minus_cdscc[0]*eye_minus_cdscc[0] + eye_minus_cdscc[1]*eye_minus_cdscc[1] + eye_minus_cdscc[2]*eye_minus_cdscc[2])

	y_direction[0] = eye_minus_cdscc[0] / norm_y_direction
	y_direction[1] = eye_minus_cdscc[1] / norm_y_direction
	// y_direction[1] = 1.0 // tends to unity
	y_direction[2] = eye_minus_cdscc[2] / norm_y_direction

	// %%     step 6: x_direction via cross product
	// %     x_direction_cp = cross(centre_direction, y_direction);

	x_direction[0] = centre_direction[1]*y_direction[2] - centre_direction[2]*y_direction[1]
	// %     x_direction[1] = centre_direction[2]*y_direction[0] - centre_direction[0]*y_direction[2]; % very close to zero
	// x_direction(2) = centre_direction(3)*y_direction(1) - centre_direction(1)*y_direction(3); % very close to zero
	x_direction[1] = centre_direction[2]*y_direction[0] - centre_direction[0]*y_direction[2]
	x_direction[2] = centre_direction[0]*y_direction[1] - centre_direction[1]*y_direction[0]

	// %% step 7: x and y corrodinates:
	// x(n) = x_direction(1)* projected(n, 1) + x_direction(2)* projected(n, 2)  + x_direction(3)* projected(n, 3) ;
	// y(n) = y_direction(1)* projected(n, 1) + y_direction(2)* projected(n, 2)  + y_direction(3)* projected(n, 3) ;

	minX := 1.0
	maxX := -1.0
	minY := minX
	maxY := maxX
	for i := 0; i < n; i++ {
		// x[i] = x_direction[0]*projected_circ_buffer[i][0] + x_direction[2]*projected_circ_buffer[i][2]                               //  % x_direction(2) is so small we can ignore it
		x[i] = x_direction[0]*projected_circ_buffer[i][0] + x_direction[1]*projected_circ_buffer[i][1] + x_direction[2]*projected_circ_buffer[i][2]
		y[i] = y_direction[0]*projected_circ_buffer[i][0] + projected_circ_buffer[i][1] + y_direction[2]*projected_circ_buffer[i][2] // ; % y_direction(2) -> 1.0
		minX = math.Min(minX, x[i])
		maxX = math.Max(maxX, x[i])
		minY = math.Min(minY, y[i])
		maxY = math.Max(maxY, y[i])
	}

	// // % scale

	absMixX := math.Abs(minX)
	if absMixX > maxX {
		maxX = absMixX
	}

	absMinY := math.Abs(minY)
	if absMinY > maxY {
		maxY = absMinY
	}

	maxdim := math.Max(maxX, maxY)
	scaler := 0.85 / maxdim // scale image so we don't extend to the edge: this REALLY help %prob!

	for i := 0; i < n; i++ {
		x[i] = x[i] * scaler
		y[i] = y[i] * scaler

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
		x_int = int(x[i]*lp/2 + lp/2)
		y_int = int(y[i]*lp/2 + lp/2)
		letterImage[y_int][x_int] = 1
		// letterImage[y_int+1][x_int+1] = 1   // make line thicker
	}

	var joinedArray []byte

	// resize matrix into long array:
	for ix := 0; ix < lp; ix++ {
		nums := letterImage[ix][:]
		joinedArray = append(joinedArray, nums...)
		// joinedArray := bytes.Join(nums, nil)  // dunno how to do this
	}

	encoded := base64.StdEncoding.EncodeToString(joinedArray)

	// -------------------------------
	// Save to out.bmp

	// NB saved as root
	// sudo chmod 777 *.bmp

	// img := image.NewRGBA(image.Rect(0, 0, lp, lp))

	// for i := 0; i < n; i++ {
	// 	x_int = int(x[i]*lp/2 + lp/2 + 1)
	// 	y_int = int(y[i]*lp/2 + lp/2 + 1)
	// 	img.Set(y_int, x_int, color.RGBA{255, 0, 0, 255})
	// }

	// sn := GetFilenameDate()

	// fo, err := os.OpenFile(sn, os.O_WRONLY|os.O_CREATE, 0600)
	// if err != nil {
	// 	log.Printf("err %s\n", err)
	// }

	// defer fo.Close()
	// bmp.Encode(fo, img)

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
