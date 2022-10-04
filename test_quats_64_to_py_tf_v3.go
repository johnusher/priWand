package main

// test_quats_64_to_py_tf_v3.go
//  uses new and improved quats2Image function
// runs on windoze or raspi
// standalone- doesnt use any hardware
// sept. 22 updated quaternion processing algo
// read in quaternion data from /letters directory, (iterate over all examples)
// process to a 28x28 array
// convert to base64
// pipe to a tensorflow lite model in python
// receive string response from tflite idicating letter match and probability

import (
	"bufio"
	"encoding/base64"
	"fmt"
	"io"
	"io/ioutil"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"runtime"
	"sort"
	"time"

	log "github.com/sirupsen/logrus"
)

const (
	circBufferL = 600 // length of buffer where we store quat data. 600 samples @5 ms update = 3 seconds
	lp          = 28  // pixels used to represent drawn letter, on each axis, ie lpxlp
	imuSkip     = 11  // skip first and last 50 samples
)

func main() {

	// var quat_inFn = "./letters/feb22/M_22-14-45/quat_data.txt"
	var quat_inFn = "./letters/M/M_20-45-40/quaternion_data.txt"
	// C:\Users\john\Documents\Arduino\priWand\priWand\letters\M\M_20-45-40

	var quat_in_circ_buffer [circBufferL][5]float64 // raw quaternion inputs from file or IMU

	// var yaw [circBufferL]float64
	// var sortedyaw [circBufferL]float64

	var n int

	searchDir := "letters"

	pattern := "quaternion_data.txt"
	// pattern := "quat_data.txt"

	fileList := make([]string, 0)
	err := filepath.Walk(searchDir, func(path string, f os.FileInfo, err error) error {
		fileList = append(fileList, path)
		return err
	})

	if err != nil {
		fmt.Println(err)
	}

	cmd := exec.Command("python3", "-u", "classifier/classify.py") // linux
	if runtime.GOOS == "windows" {
		cmd = exec.Command("python", "-u", "classifier/classify.py") // windoze
	}

	stdout, err := cmd.StdoutPipe()
	if err != nil {
		panic(err)
	}

	stdin, err := cmd.StdinPipe()
	if err != nil {
		panic(err)
	}

	stdoutReader := bufio.NewReader(stdout)

	err = cmd.Start()
	if err != nil {
		panic(err)
	}

	n = 0

	for _, file := range fileList {
		// for _, file := range fileList {

		matched, _ := regexp.MatchString(pattern, file)
		// fmt.Println(matched, err)

		if matched {
			log.Printf(" file: %v", file)
			// quat_inFn = file  // comment ot just load one

			log.Printf(" quat_inFn: %v", quat_inFn)

			f, err := os.Open(quat_inFn)
			if err != nil {
				fmt.Println(err)
			}

			n = 0 // index to write into circ buffer
			for {

				var flt1, flt2, flt3, flt4 float64

				if n == 0 {
					for i := 0; i < imuSkip; i++ {
						fn, err := fmt.Fscan(f, &flt1, &flt2, &flt3, &flt4)
						if fn == 0 || err != nil {
							break
						}
					}
				} else {
					fn, err := fmt.Fscan(f, &flt1, &flt2, &flt3, &flt4)
					if fn == 0 || err != nil {
						break
					}

				}

				// fn, err := fmt.Fscan(f, &flt1, &flt2, &flt3, &flt4)
				// if fn == 0 || err != nil {
				// 	break
				// }

				quat_in_circ_buffer[n][0] = flt1
				quat_in_circ_buffer[n][1] = flt2
				quat_in_circ_buffer[n][2] = flt3
				quat_in_circ_buffer[n][3] = flt4

				n = n + 1

			}

			n = n - 3

			encoded, _ := quats2Image(quat_in_circ_buffer, n)

			// now send to the python:

			now1 := time.Now()

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

			now2 := time.Now()
			elapsedTime := now2.Sub(now1)
			log.Printf("elapsedTime TF=%v", elapsedTime)
			log.Printf("raw message: %v", s2)

			// // print first and second place:
			// s := strings.FieldsFunc(s2, Split)

			// prob, _ := strconv.ParseFloat(s[0], 64)
			// // letter := strings.Trim(s[1], "'")
			// letter := strings.Replace(s[1], "'", "", -1)

			// // s[2] is blank
			// prob2, _ := strconv.ParseFloat(s[3], 64)
			// letter2 := strings.Replace(s[4], "'", "", -1)

			// log.Printf("letter1: %v", letter)
			// log.Printf("prob1: %v", prob)

			// log.Printf("letter2: %v", letter2)
			// log.Printf("prob2: %v", prob2)

			// now2 := time.Now()
			// elapsedTime = now2.Sub(now1)

			// takes bout 1.5 ms to save bmp, 10 ms to save png
			// log.Printf("elapsedTime2=%v", elapsedTime)

			// break

		}

	}

}

// func request(r *bufio.Reader, w io.Writer, str string) string {
// 	w.Write([]byte(str))
// 	w.Write([]byte("\n"))
// 	str, err := r.ReadString('\n')
// 	if err != nil {
// 		panic(err)
// 	}
// 	return str[:len(str)-1]
// }

func copyOutput(r io.Reader) {
	scanner := bufio.NewScanner(r)
	for scanner.Scan() {
		fmt.Println(scanner.Text())
	}
}

func ReadOutput(rc io.ReadCloser) (string, error) {
	x, err := ioutil.ReadAll(rc)
	s := string(x)
	return s, err
}

func ReadOutput2(output chan string, rc io.ReadCloser) {
	r := bufio.NewReader(rc)
	for {
		x, _ := r.ReadString('\n')
		output <- string(x)
	}
}

func sliceToInt(s []byte) byte {
	res := int(0)
	op := int(1)
	for i := len(s) - 1; i >= 0; i-- {
		res += int(s[i]) * op
		op *= 2

		// log.Printf(" res: %v", res)

	}
	return byte(res)
}

func Split(r rune) bool {
	return r == ':' || r == ',' || r == '(' || r == ')' || r == '[' || r == ']'
}

func quats2Image(quat_in_circ_buffer [circBufferL][5]float64, length int) (string, [lp][lp]byte) {
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

	for n = 0; n < length; n++ {
		s := quat_in_circ_buffer[n][0]
		x := quat_in_circ_buffer[n][1]
		y := quat_in_circ_buffer[n][2]
		z := quat_in_circ_buffer[n][3]

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

	min_yaw := 10.0
	max_yaw := -10.0
	min_pitch := 10.0
	max_pitch := -10.0
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
		letterImage[y_int][x_int] = 1
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
	// img := image.NewRGBA(image.Rect(0, 0, lp, lp))

	// x_int := 0
	// y_int := 0
	// for i := 0; i < n; i++ {
	// 	// x_int = lp - int(yaw[i]*lp)
	// 	// y_int = lp - int(pitch[i]*lp)

	// 	x_int = int(yaw[i] * lp)
	// 	y_int = lp - int(pitch[i]*lp) - 1

	// 	img.Set(x_int, y_int, color.RGBA{255, 0, 0, 255})
	// }

	// sn := strings.Replace(file, "quaternion_data.txt", "quat_image.bmp", 1)
	// // sn := strings.Replace(file, "quat_data.txt", "quat_image.bmp", 1)

	// fo, err := os.OpenFile(sn, os.O_WRONLY|os.O_CREATE, 0600)
	// if err != nil {
	// 	log.Printf("err %s\n", err)
	// }

	// defer fo.Close()
	// bmp.Encode(fo, img)

	return encoded, letterImage
}
